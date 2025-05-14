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
from typing import Optional
from typing import Tuple

import cv2
import cv_bridge
import mediapipe
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

from oasis_msgs.msg import BoundingBox as BoundingBoxMsg
from oasis_msgs.msg import CameraScene as CameraSceneMsg


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
IMAGE_PUB_TOPIC = "pose_landmarks"

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
    Smooth bounding box coordinates with adaptive responsiveness,
    and then apply padding.
    """

    def __init__(
        self,
        *,
        base_alpha: float = 0.3,
        padding: int = 10,
        velocity_threshold_px: float = 20.0,
    ) -> None:
        """
        Initialize the bounding box smoother.

        :param base_alpha: Base smoothing factor in [0..1] (lower is smoother)
        :param padding: Pixel padding around the smoothed box
        :param velocity_threshold_px: Displacement (px) that triggers full α=1
        """
        self.base_alpha = base_alpha
        self.padding = padding
        self.velocity_threshold = velocity_threshold_px

        # Remember the last smoothed box (x_min, y_min, x_max, y_max)
        self._prev_box: Optional[Tuple[float, float, float, float]] = None

    def update(self, box: Tuple[int, int, int, int]) -> Tuple[int, int, int, int]:
        """
        Smooth the incoming bounding box and return a padded integer box.

        :param box: Raw bounding box (x_min, y_min, x_max, y_max)

        :return: Smoothed + padded bounding box as ints
        """
        x0, y0, x1, y1 = (float(c) for c in box)

        # Calculate raw center and half‐sizes
        cx = (x0 + x1) * 0.5
        cy = (y0 + y1) * 0.5
        hw = (x1 - x0) * 0.5
        hh = (y1 - y0) * 0.5

        if self._prev_box is None:
            # First frame: no smoothing
            scx, scy, shw, shh = cx, cy, hw, hh
        else:
            px0, py0, px1, py1 = self._prev_box
            pcx = (px0 + px1) * 0.5
            pcy = (py0 + py1) * 0.5
            phw = (px1 - px0) * 0.5
            phh = (py1 - py0) * 0.5

            # Compute displacement of center
            disp = math.hypot(cx - pcx, cy - pcy)

            # If fast (disp > threshold), jump faster (α→1); else use base α
            if disp >= self.velocity_threshold:
                alpha = 1.0
            else:
                # Scale α linearly between base and 1 as disp → threshold
                alpha = self.base_alpha + (1.0 - self.base_alpha) * (
                    disp / self.velocity_threshold
                )

            # Smooth each component
            scx = alpha * cx + (1 - alpha) * pcx
            scy = alpha * cy + (1 - alpha) * pcy
            shw = alpha * hw + (1 - alpha) * phw
            shh = alpha * hh + (1 - alpha) * phh

        # Store for next time (reconstruct full box)
        nx0 = scx - shw
        ny0 = scy - shh
        nx1 = scx + shw
        ny1 = scy + shh
        self._prev_box = (nx0, ny0, nx1, ny1)

        # Apply padding and round
        x_min = int(nx0 - self.padding)
        y_min = int(ny0 - self.padding)
        x_max = int(nx1 + self.padding)
        y_max = int(ny1 + self.padding)

        return (x_min, y_min, x_max, y_max)


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
        self._image_sub = ImageTransport(
            self.get_name() + "_sub", image_transport="compressed"
        )
        self._image_sub.subscribe(
            self.resolve_topic_name(IMAGE_SUB_TOPIC), 1, self._image_callback
        )

        # Publisher: advertise the annotated image using the "compressed" transport
        self._image_pub = ImageTransport(
            self.get_name() + "_pub", image_transport="compressed"
        )
        self._pub = self._image_pub.advertise(
            self.resolve_topic_name(IMAGE_PUB_TOPIC), 1
        )

        # Camera scene publisher
        self._scene_pub = self.create_publisher(
            CameraSceneMsg,
            self.resolve_topic_name(CAMERA_SCENE_TOPIC),
            1,
        )

        # Initialize MediaPipe pose detector in IMAGE mode for synchronous processing
        self._detector = self.initialize_detector()

        # Setup MediaPipe drawing utilities
        self._mp_drawing = mediapipe.solutions.drawing_utils
        self._mp_drawing_styles = mediapipe.solutions.drawing_styles
        self._mp_pose = mediapipe.solutions.pose

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

    def _image_callback(self, msg: sensor_msgs.msg.Image) -> None:
        if not msg.data:
            self.get_logger().error("Received empty image message")
            return

        # Grab the header stamp
        header_stamp = msg.header.stamp
        if header_stamp.sec != 0 or header_stamp.nanosec != 0:
            # Use camera driver’s timestamp
            timestamp_ms = header_stamp.sec * 1000 + (header_stamp.nanosec // 1_000_000)
        else:
            # Fall back to now()
            now_ns = self.get_clock().now().nanoseconds
            timestamp_ms = int(now_ns // 1e6)

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
        mp_image = mediapipe.Image(
            image_format=mediapipe.ImageFormat.SRGB, data=rgb_image
        )

        # Submit the frame for asynchronous processing
        self._detector.detect_async(mp_image, timestamp_ms)

    def _result_callback(self, result, output_image, timestamp_ms) -> None:
        """
        Called asynchronously when the detector finishes processing a frame. The
        output_image is a mediapipe.Image. We convert it to a NumPy array using
        copy_to_numpy_view().
        """
        try:
            # Convert the mediapipe.Image to a NumPy array
            annotated_image = output_image.numpy_view().copy()
        except Exception as e:
            self.get_logger().error("Failed to convert mediapipe image: " + str(e))
            return

        # Lookup the original header
        header: HeaderMsg | None = self._stamp_map.pop(timestamp_ms, None)
        if header is None:
            # If it wasn't stored (unlikely), reconstruct from timestamp_ms
            secs = timestamp_ms // 1000
            nsecs = int((timestamp_ms % 1000) * 1e6)

            header = HeaderMsg()
            header.stamp = TimeMsg(sec=secs, nanosec=nsecs)

        # Keep track of the bounding boxes for each detected pose
        boxes: list[BoundingBoxMsg] = []

        # If landmarks are detected, draw them on the original image
        if result.pose_landmarks:
            # Log the number of detected poses
            if self._pose_count != len(result.pose_landmarks):
                self._pose_count = len(result.pose_landmarks)
                self.get_logger().debug(f"Detected {len(result.pose_landmarks)} poses")

            # A list of bounding boxes for each detected pose
            boxes: list[BoundingBoxMsg] = []

            for i, landmarks in enumerate(result.pose_landmarks):
                # Create a NormalizedLandmarkList message
                landmark_list = landmark_pb2.NormalizedLandmarkList()
                landmark_list.landmark.extend(
                    [
                        landmark_pb2.NormalizedLandmark(x=lm.x, y=lm.y, z=lm.z)
                        for lm in landmarks
                    ]
                )
                self._mp_drawing.draw_landmarks(
                    annotated_image,
                    landmark_list,
                    self._mp_pose.POSE_CONNECTIONS,
                    self._mp_drawing_styles.get_default_pose_landmarks_style(),
                )

                # Draw bounding box around each pose
                h, w, _ = annotated_image.shape
                xs = [int(lm.x * w) for lm in landmarks]
                ys = [int(lm.y * h) for lm in landmarks]
                raw_box = (min(xs), min(ys), max(xs), max(ys))
                smooth_box = self._bbox_smoothers[i].update(raw_box)

                # Compute pixel dimensions
                px_c = (smooth_box[0] + smooth_box[2]) / 2.0
                py_c = (smooth_box[1] + smooth_box[3]) / 2.0
                pw = smooth_box[2] - smooth_box[0]
                ph = smooth_box[3] - smooth_box[1]

                # Normalize to [0..1]
                nx = px_c / w
                ny = py_c / h
                nw = pw / w
                nh = ph / h

                # Fill and publish bounding‐box message
                bbox_msg = BoundingBoxMsg()
                bbox_msg.x_center = nx
                bbox_msg.y_center = ny
                bbox_msg.width = nw
                bbox_msg.height = nh
                boxes.append(bbox_msg)

                cv2.rectangle(
                    annotated_image,
                    (smooth_box[0], smooth_box[1]),
                    (smooth_box[2], smooth_box[3]),
                    (0, 255, 255),  # Cyan in RGB
                    thickness=3,
                )
        else:
            # Log when no poses are detected
            if self._pose_count > 0:
                self._pose_count = 0
                self.get_logger().debug("No poses detected")

        # Publish the scene (all boxes in one message)
        scene = CameraSceneMsg()
        scene.header = header
        scene.bounding_boxes = boxes
        self._scene_pub.publish(scene)

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
        self._pub.publish(ros_image_msg)

    def stop(self) -> None:
        self.get_logger().info("Pose landmarker node shutting down")
        self.destroy_node()
