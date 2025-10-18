################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import math
import os
from typing import Any, Optional, cast

import cv2
import cv_bridge
import mediapipe
import numpy as np
import rclpy.node
import sensor_msgs.msg
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from image_transport_py import ImageTransport
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Header as HeaderMsg

from oasis_msgs.msg import BoundingBox as BoundingBoxMsg  # type: ignore[attr-defined]
from oasis_msgs.msg import CameraScene as CameraSceneMsg  # type: ignore[attr-defined]
from oasis_msgs.msg import Landmark as LandmarkMsg  # type: ignore[attr-defined]
from oasis_msgs.msg import PoseLandmarks as PoseLandmarksMsg  # type: ignore[attr-defined]
from oasis_msgs.msg import PoseLandmarksArray as PoseLandmarksArrayMsg  # type: ignore[attr-defined]

mediapipe_module = cast(Any, mediapipe)


################################################################################
# ROS parameters
################################################################################


# ROS package name
PACKAGE_NAME = "oasis_perception_py"

# Default node name
NODE_NAME = "pose_landmarker"

# Subscribers
IMAGE_SUB_TOPIC = "image"

# Publishers
CAMERA_SCENE_TOPIC = "camera_scene"
POSE_IMAGE_TOPIC = "pose_image"
POSE_LANDMARKS_TOPIC = "pose"

################################################################################
# Pose landmarker parameters
################################################################################


# Number of poses to detect
NUM_POSES = 5


################################################################################
# Bounding box class
################################################################################


class BoundingBoxSmoother:
    """
    Smooth bounding boxes and apply normalized padding.

    All inputs and outputs are in normalized [0..1] coordinates.
    """

    def __init__(
        self,
        *,
        base_alpha: float = 0.3,
        padding: float = 0.02,
        velocity_threshold: float = 0.05,
    ) -> None:
        """
        Initialize the smoother.

        :param base_alpha: Base smoothing factor ∈ [0..1]. Lower = smoother.
        :param padding: Normalized padding to apply on each side of the box.
        :param velocity_threshold: Normalized center‐to‐center movement that
                                   triggers α→1.0 (no lag).
        """
        self.base_alpha: float = base_alpha
        self.padding: float = padding
        self.velocity_threshold: float = velocity_threshold
        self._prev_box: Optional[tuple[float, float, float, float]] = None

    def update(
        self,
        raw_box: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        """
        Smooth the raw box and return a padded, smoothed box, all in [0..1].

        :param raw_box: (x_min, y_min, x_max, y_max) in normalized coords.
        :return: (x_min, y_min, x_max, y_max) smoothed & padded, clamped to [0..1].
        """
        # Unpack
        x0, y0, x1, y1 = raw_box
        # Compute center + half‐widths
        cx = (x0 + x1) * 0.5
        cy = (y0 + y1) * 0.5
        hw = (x1 - x0) * 0.5
        hh = (y1 - y0) * 0.5

        # First frame: no smoothing
        if self._prev_box is None:
            scx, scy, shw, shh = cx, cy, hw, hh
        else:
            # Unpack previous
            px0, py0, px1, py1 = self._prev_box
            pcx = (px0 + px1) * 0.5
            pcy = (py0 + py1) * 0.5
            phw = (px1 - px0) * 0.5
            phh = (py1 - py0) * 0.5

            # Measure normalized velocity
            disp = math.hypot(cx - pcx, cy - pcy)

            # Adapt alpha: no lag if disp ≥ threshold
            if disp >= self.velocity_threshold:
                alpha = 1.0
            else:
                alpha = self.base_alpha + (1 - self.base_alpha) * (
                    disp / self.velocity_threshold
                )

            # Smooth each component
            scx = alpha * cx + (1 - alpha) * pcx
            scy = alpha * cy + (1 - alpha) * pcy
            shw = alpha * hw + (1 - alpha) * phw
            shh = alpha * hh + (1 - alpha) * phh

        # Store for next call
        nx0 = scx - shw
        ny0 = scy - shh
        nx1 = scx + shw
        ny1 = scy + shh
        self._prev_box = (nx0, ny0, nx1, ny1)

        # Apply normalized padding
        px0 = nx0 - self.padding
        py0 = ny0 - self.padding
        px1 = nx1 + self.padding
        py1 = ny1 + self.padding

        # Clamp to [0..1]
        x_min = max(0.0, px0)
        y_min = max(0.0, py0)
        x_max = min(1.0, px1)
        y_max = min(1.0, py1)

        return (x_min, y_min, x_max, y_max)

    def get_smoothed_box(self) -> tuple[float, float, float, float]:
        """
        Return the last smoothed & padded box (normalized), without re-smoothing.
        """
        if self._prev_box is None:
            raise RuntimeError("No previous box: call update() first.")
        raw_box = self._prev_box
        # Reapply padding & clamp
        px0 = raw_box[0] - self.padding
        py0 = raw_box[1] - self.padding
        px1 = raw_box[2] + self.padding
        py1 = raw_box[3] + self.padding
        return (
            max(0.0, px0),
            max(0.0, py0),
            min(1.0, px1),
            min(1.0, py1),
        )


################################################################################
# ROS node
################################################################################


class PoseLandmarkerNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.get_logger().info("Pose landmarker node initializing")

        # Pose detection state
        self._pose_count: int = 0

        # Initialize bounding box smoothers for each pose
        self._bbox_smoothers: list[BoundingBoxSmoother] = [
            BoundingBoxSmoother() for _ in range(NUM_POSES)
        ]

        # Last timestamp passed to MediaPipe. Must be strictly increasing.
        self._last_timestamp_ms: int = 0

        # Map from timestamp_ms -> original header stamp
        self._stamp_map: dict[int, HeaderMsg] = {}

        # Initialize cv_bridge to convert between ROS and OpenCV images
        self._cv_bridge = cv_bridge.CvBridge()

        # Create an ImageTransport object using this node's name and specifying
        # the 'compressed' transport
        self._image_transport: ImageTransport = ImageTransport(
            self.get_name(), image_transport="compressed"
        )

        # Subscribers
        self._image_transport.subscribe(
            self.resolve_topic_name(IMAGE_SUB_TOPIC), 1, self._image_callback
        )

        # Publishers
        self._scene_pub = self.create_publisher(
            CameraSceneMsg,
            self.resolve_topic_name(CAMERA_SCENE_TOPIC),
            1,
        )
        self._pose_image_pub = self._image_transport.advertise(
            self.resolve_topic_name(POSE_IMAGE_TOPIC), 1
        )
        self._pose_landmarks_pub = self.create_publisher(
            PoseLandmarksArrayMsg,
            POSE_LANDMARKS_TOPIC,
            1,
        )

        # Initialize MediaPipe pose detector in IMAGE mode for synchronous processing
        self._detector = self.initialize_detector()

        # Setup MediaPipe drawing utilities
        self._mp_drawing = mediapipe_module.solutions.drawing_utils
        self._mp_drawing_styles = mediapipe_module.solutions.drawing_styles
        self._mp_pose = mediapipe_module.solutions.pose

    def initialize_detector(self):
        # Get model_asset_path as needed
        model_path = os.path.join(
            get_package_share_directory(PACKAGE_NAME),
            "mediapipe",
            "pose_landmarker.task",
        )
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM,
            num_poses=NUM_POSES,
            min_pose_detection_confidence=0.75,
            min_pose_presence_confidence=0.4,
            min_tracking_confidence=0.4,
            result_callback=self._result_callback,
        )
        return vision.PoseLandmarker.create_from_options(options)

    def stop(self) -> None:
        self.get_logger().info("Pose landmarker node shutting down")
        self.destroy_node()

    def _image_callback(self, msg: sensor_msgs.msg.Image) -> None:
        if not msg.data:
            self.get_logger().error("Received empty image message")
            return

        # Grab the header stamp
        header_stamp = msg.header.stamp
        if header_stamp.sec != 0 or header_stamp.nanosec != 0:
            # Use camera driver’s timestamp
            timestamp_ms: int = header_stamp.sec * 1000 + (
                header_stamp.nanosec // 1_000_000
            )
        else:
            # Using the current time might break strict monotonicity, so bail
            # out if the camera driver doesn't provide a timestamp
            self.get_logger().error(
                "Camera driver did not provide a timestamp. Cannot process image."
            )
            return

        # Enforce strictly monotonic increase
        if timestamp_ms <= self._last_timestamp_ms:
            self.get_logger().error(
                f"Timestamp {timestamp_ms} is not strictly increasing from last {self._last_timestamp_ms}"
            )
            return
        self._last_timestamp_ms = timestamp_ms

        # Store for later use in the result callback
        self._stamp_map[timestamp_ms] = msg.header

        try:
            # Convert the raw ROS image to an OpenCV image with rgb8 encoding
            rgb_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error("Image conversion failed: " + str(e))
            return

        # Create a MediaPipe image from the RGB data (no further color conversion required)
        mp_image: Any = mediapipe_module.Image(
            image_format=mediapipe_module.ImageFormat.SRGB, data=rgb_image
        )

        # Submit the frame for asynchronous processing
        self._detector.detect_async(mp_image, timestamp_ms)

    def _result_callback(
        self,
        result: vision.PoseLandmarkerResult,
        output_image: Any,
        timestamp_ms: int,
    ) -> None:
        """
        Called asynchronously when the detector finishes processing a frame.

        The output_image is a mediapipe.Image. We convert it to a NumPy array.
        """
        # Log detected poses
        if result.pose_landmarks:
            # Log the number of detected poses
            if self._pose_count != len(result.pose_landmarks):
                self._pose_count = len(result.pose_landmarks)
                self.get_logger().debug(f"Detected {len(result.pose_landmarks)} poses")
        else:
            # Log when no poses are detected
            if self._pose_count > 0:
                self._pose_count = 0
                self.get_logger().debug("No poses detected")

        # Lookup the original header
        header: HeaderMsg | None = self._stamp_map.pop(timestamp_ms, None)
        if header is None:
            # If it wasn't stored (unlikely), reconstruct from timestamp_ms
            secs: int = timestamp_ms // 1000
            nsecs: int = int((timestamp_ms % 1000) * 1e6)

            header = HeaderMsg()
            header.stamp = TimeMsg(sec=secs, nanosec=nsecs)

        self._publish_pose_messages(result, header)

        self._publish_image(result, output_image, header)

    def _publish_pose_messages(
        self, result: vision.PoseLandmarkerResult, header: HeaderMsg
    ) -> None:
        """
        Publish pose landmarks and bounding boxes.
        :param result: The result from the pose landmarker.
        :param header: The header to use for the published messages.
        """
        # Build and publish pose landmark messages
        pose_array_msg: PoseLandmarksArrayMsg = PoseLandmarksArrayMsg()
        scene_msg: CameraSceneMsg = CameraSceneMsg()

        pose_array_msg.header = header
        scene_msg.header = header

        i: int
        landmarks: list[landmark_pb2.NormalizedLandmarkList]
        for i, landmarks in enumerate(result.pose_landmarks or []):
            pose_msg: PoseLandmarksMsg = PoseLandmarksMsg()
            for landmark in landmarks:
                pose_msg.landmarks.append(
                    LandmarkMsg(x=landmark.x, y=landmark.y, z=landmark.z)
                )
            pose_array_msg.poses.append(pose_msg)

            # Compute raw normalized bounding box from landmark coords
            xs: list[float] = [landmark.x for landmark in landmarks]
            ys: list[float] = [landmark.y for landmark in landmarks]
            raw_box: tuple[float, float, float, float] = (
                min(xs),  # x_min
                min(ys),  # y_min
                max(xs),  # x_max
                max(ys),  # y_max
            )

            x0_n: float
            y0_n: float
            x1_n: float
            y1_n: float
            x0_n, y0_n, x1_n, y1_n = self._bbox_smoothers[i].update(raw_box)

            # 3) Append normalized box to CameraSceneMsg
            bbox_norm: BoundingBoxMsg = BoundingBoxMsg()
            bbox_norm.x_center = (x0_n + x1_n) * 0.5
            bbox_norm.y_center = (y0_n + y1_n) * 0.5
            bbox_norm.width = x1_n - x0_n
            bbox_norm.height = y1_n - y0_n
            scene_msg.bounding_boxes.append(bbox_norm)
        self._pose_landmarks_pub.publish(pose_array_msg)
        self._scene_pub.publish(scene_msg)

    def _publish_image(
        self,
        result: vision.PoseLandmarkerResult,
        output_image: Any,
        header: HeaderMsg,
    ) -> None:
        # Convert the mediapipe.Image to a NumPy array
        try:
            annotated_image: np.ndarray = output_image.numpy_view().copy()
        except Exception as e:
            self.get_logger().error("Failed to convert mediapipe image: " + str(e))
            return

        # Get image dimensions
        h: int
        w: int
        h, w, _ = annotated_image.shape

        for i, landmarks in enumerate(result.pose_landmarks or []):
            # Draw landmarks
            landmark_list: landmark_pb2.NormalizedLandmarkList = (
                landmark_pb2.NormalizedLandmarkList()
            )
            landmark_list.landmark.extend(
                [
                    landmark_pb2.NormalizedLandmark(
                        x=landmark.x, y=landmark.y, z=landmark.z
                    )
                    for landmark in landmarks
                ]
            )
            self._mp_drawing.draw_landmarks(
                annotated_image,
                landmark_list,
                self._mp_pose.POSE_CONNECTIONS,
                self._mp_drawing_styles.get_default_pose_landmarks_style(),
            )

            # Get the last smoothed & padded box in normalized coords
            x0_n, y0_n, x1_n, y1_n = self._bbox_smoothers[i].get_smoothed_box()

            # Convert to pixel coordinates
            px0: int = int(x0_n * w)
            py0: int = int(y0_n * h)
            px1: int = int(x1_n * w)
            py1: int = int(y1_n * h)

            # Draw the rectangle
            cv2.rectangle(
                annotated_image,
                (px0, py0),
                (px1, py1),
                (0, 255, 255),  # Cyan in RGB
                thickness=3,
            )

        # Convert the annotated image back to a ROS Image message
        try:
            ros_image_msg = self._cv_bridge.cv2_to_imgmsg(
                annotated_image, encoding="rgb8"
            )
        except Exception as err:
            # If conversion fails, we can’t publish an image header, so bail
            self.get_logger().error(
                "Failed to convert annotated image to ROS msg: " + str(err)
            )
            return

        # Set the header for the annotated image
        ros_image_msg.header = header

        # Publish the annotated image
        self._pose_image_pub.publish(ros_image_msg)
