################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import os
from dataclasses import dataclass
from threading import Lock
from typing import Any

import cv2
import cv_bridge
import numpy as np
import rclpy.node
from ament_index_python import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from image_transport_py import ImageTransport
from mediapipe import Image as MediapipeImage  # type: ignore[attr-defined]
from mediapipe import ImageFormat as MediapipeImageFormat  # type: ignore[attr-defined]
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from oasis_perception.utils.bounding_box_smoother import BoundingBoxSmoother
from oasis_perception.utils.pose_drawing import draw_pose_landmarks
from rclpy.logging import LoggingSeverity
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_default
from rclpy.timer import Timer
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header as HeaderMsg

from oasis_msgs.msg import BoundingBox as BoundingBoxMsg
from oasis_msgs.msg import CameraScene as CameraSceneMsg
from oasis_msgs.msg import Landmark as LandmarkMsg
from oasis_msgs.msg import PoseLandmarks as PoseLandmarksMsg
from oasis_msgs.msg import PoseLandmarksArray as PoseLandmarksArrayMsg


################################################################################
# ROS parameters
################################################################################


# ROS package name
PACKAGE_NAME = "oasis_perception_py"

# Default node name
NODE_NAME = "pose_landmarker"

# Parameters
IMAGE_TRANSPORT_PARAM = "image_transport"
IMAGE_TRANSPORT_DEFAULT = "compressed"
PUBLISH_POSE_IMAGE_PARAM = "publish_pose_image"
PUBLISH_POSE_IMAGE_DEFAULT = True

# Subscribers
IMAGE_SUB_TOPIC = "image"

# Publishers
CAMERA_SCENE_TOPIC = "camera_scene"
POSE_IMAGE_TOPIC = "pose_image"
POSE_LANDMARKS_TOPIC = "pose_landmarks"

################################################################################
# Pose landmarker parameters
################################################################################


# Number of poses to detect
NUM_POSES = 5

# Maximum timestamp/header pairs retained for in-flight MediaPipe results
STAMP_MAP_MAX_SIZE = 4


@dataclass
class PreparedFrame:
    """
    Frame converted for MediaPipe live-stream processing.

    timestamp_ms: ROS header timestamp in milliseconds, strictly increasing
    header: Original ROS header used for publishing result messages
    image_msg: Source image backing any zero-copy NumPy or MediaPipe views
    mp_image: MediaPipe image in an encoding accepted by the detector
    """

    timestamp_ms: int
    header: HeaderMsg
    image_msg: ImageMsg
    mp_image: MediapipeImage


@dataclass
class RenderJob:
    """
    Annotated pose image render work queued outside the detector result path.

    result: MediaPipe pose result used for landmark overlay drawing
    output_image: MediaPipe RGB image returned with the detector result
    header: Original ROS header used for the annotated image message
    """

    result: vision.PoseLandmarkerResult
    output_image: MediapipeImage
    header: HeaderMsg


################################################################################
# ROS node
################################################################################


class PoseLandmarkerNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.get_logger().info("Pose landmarker node initializing")

        # Declare parameters
        self.declare_parameter(IMAGE_TRANSPORT_PARAM, IMAGE_TRANSPORT_DEFAULT)
        self.declare_parameter(PUBLISH_POSE_IMAGE_PARAM, PUBLISH_POSE_IMAGE_DEFAULT)

        image_transport_param: str = (
            self.get_parameter(IMAGE_TRANSPORT_PARAM).get_parameter_value().string_value
        )
        image_transport: str = image_transport_param or IMAGE_TRANSPORT_DEFAULT
        self._publish_pose_image_enabled: bool = (
            self.get_parameter(PUBLISH_POSE_IMAGE_PARAM)
            .get_parameter_value()
            .bool_value
        )

        # Pose detection state
        self._pose_count: int = 0

        # Initialize bounding box smoothers for each pose
        self._bbox_smoothers: list[BoundingBoxSmoother] = [
            BoundingBoxSmoother() for _ in range(NUM_POSES)
        ]

        # Detector scheduling state. At most one detection is in MediaPipe and
        # at most one newest prepared frame waits locally.
        self._detector_lock = Lock()
        self._detector_busy: bool = False
        self._pending_frame: PreparedFrame | None = None
        self._last_submitted_timestamp_ms: int = 0
        self._dropped_pending_frames: int = 0

        # Annotated image rendering state. Keep only the newest render job so
        # debug publishing cannot build an unbounded queue.
        self._render_lock = Lock()
        self._pending_render_job: RenderJob | None = None
        self._dropped_render_jobs: int = 0

        # Map from timestamp_ms -> original header
        self._stamp_map: dict[int, HeaderMsg] = {}

        # Initialize cv_bridge to convert between ROS and OpenCV images
        self._cv_bridge: cv_bridge.CvBridge = cv_bridge.CvBridge()

        # Create an ImageTransport object using this node's name and the
        # configured transport. Keep it for publishing the annotated image.
        self._image_transport: ImageTransport = ImageTransport(
            self.get_name(), image_transport=image_transport
        )

        # Subscribe directly to the raw image. PoseLandmarker does not use
        # CameraInfo, and the image_transport camera sync path can fail before
        # this node's callback is reached.
        image_qos: QoSProfile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._image_subscription = self.create_subscription(
            ImageMsg,
            self.resolve_topic_name(IMAGE_SUB_TOPIC),
            self._image_callback,
            image_qos,
        )

        # Publishers
        self._scene_pub = self.create_publisher(
            CameraSceneMsg,
            self.resolve_topic_name(CAMERA_SCENE_TOPIC),
            qos_profile_default,
        )
        self._pose_image_pub = self._image_transport.advertise(
            base_topic=self.resolve_topic_name(POSE_IMAGE_TOPIC),
            queue_size=1,
        )
        self._pose_landmarks_pub = self.create_publisher(
            PoseLandmarksArrayMsg,
            POSE_LANDMARKS_TOPIC,
            qos_profile_default,
        )
        self._render_timer: Timer | None = None
        if self._publish_pose_image_enabled:
            self._render_timer = self.create_timer(0.01, self._render_pending_image)

        # Initialize MediaPipe pose detector for live-stream processing
        self._detector = self.initialize_detector()

    def initialize_detector(self) -> vision.PoseLandmarker:
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
        if self._render_timer is not None:
            self._render_timer.cancel()
            self._render_timer = None
        with self._render_lock:
            self._pending_render_job = None
        self.destroy_node()

    def _image_callback(self, image_msg: ImageMsg) -> None:
        frame = self._prepare_frame(image_msg)
        if frame is None:
            return

        with self._detector_lock:
            if self._detector_busy:
                if self._pending_frame is not None:
                    self._dropped_pending_frames += 1
                    if self._dropped_pending_frames % 100 == 0:
                        self.get_logger().debug(
                            "Dropped "
                            f"{self._dropped_pending_frames} pending pose frames"
                        )
                self._pending_frame = frame
                return

            self._detector_busy = True

        if not self._submit_prepared_frame(frame):
            self._submit_pending_frame_or_mark_idle()

    def _timestamp_ms_from_header(self, header: HeaderMsg) -> int:
        stamp = header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            raise ValueError("Camera driver did not provide a timestamp")

        return stamp.sec * 1000 + stamp.nanosec // 1_000_000

    def _make_header_from_timestamp(self, timestamp_ms: int) -> HeaderMsg:
        secs: int = timestamp_ms // 1000
        nsecs: int = (timestamp_ms % 1000) * 1_000_000

        header = HeaderMsg()
        header.stamp = TimeMsg(sec=secs, nanosec=nsecs)
        return header

    def _prepare_frame(self, image_msg: ImageMsg) -> PreparedFrame | None:
        if not image_msg.data:
            self.get_logger().error("Received empty image message")
            return None

        try:
            timestamp_ms: int = self._timestamp_ms_from_header(image_msg.header)
        except ValueError as err:
            self.get_logger().error(str(err) + ". Cannot process image.")
            return None

        try:
            mp_image = self._ros_image_to_mediapipe_image(image_msg)
        except Exception as err:
            self.get_logger().error("Image conversion failed: " + str(err))
            return None

        return PreparedFrame(
            timestamp_ms=timestamp_ms,
            header=image_msg.header,
            image_msg=image_msg,
            mp_image=mp_image,
        )

    def _submit_prepared_frame(self, frame: PreparedFrame) -> bool:
        if frame.timestamp_ms <= self._last_submitted_timestamp_ms:
            self.get_logger().debug(
                f"Timestamp {frame.timestamp_ms} is not strictly increasing "
                f"from last {self._last_submitted_timestamp_ms}"
            )
            return False

        self._last_submitted_timestamp_ms = frame.timestamp_ms
        self._stamp_map[frame.timestamp_ms] = frame.header

        while len(self._stamp_map) > STAMP_MAP_MAX_SIZE:
            oldest_timestamp_ms: int = next(iter(self._stamp_map))
            self._stamp_map.pop(oldest_timestamp_ms, None)

        try:
            self._detector.detect_async(frame.mp_image, frame.timestamp_ms)
        except Exception as err:
            self._stamp_map.pop(frame.timestamp_ms, None)
            self.get_logger().error("MediaPipe detect_async failed: " + str(err))
            return False

        return True

    def _submit_pending_frame_or_mark_idle(self) -> None:
        while True:
            with self._detector_lock:
                next_frame: PreparedFrame | None = self._pending_frame
                self._pending_frame = None

                if next_frame is None:
                    self._detector_busy = False
                    return

            if self._submit_prepared_frame(next_frame):
                return

    @staticmethod
    def _get_mediapipe_image_format(encoding: str) -> MediapipeImageFormat:
        """
        Map ROS image encoding strings to MediaPipe image formats.
        :param encoding: The ROS image encoding string.
        :return: The corresponding MediaPipe image format.
        """
        if encoding == "rgb8":
            return MediapipeImageFormat.SRGB
        elif encoding == "rgba8":
            return MediapipeImageFormat.SRGBA
        elif encoding == "bgra8":
            return MediapipeImageFormat.SBGRA
        elif encoding == "mono8":
            return MediapipeImageFormat.GRAY8
        elif encoding == "mono16":
            return MediapipeImageFormat.GRAY16
        else:
            raise ValueError(f"Unsupported image encoding: {encoding}")

    @staticmethod
    def _get_numpy_encoding_info(encoding: str) -> tuple[np.dtype, int]:
        """
        Return the NumPy dtype and number of channels for a ROS image encoding.
        :param encoding: The ROS image encoding string.
        :return: Tuple of (dtype, channels).
        """
        if encoding in ("rgb8", "bgr8"):
            return (np.dtype(np.uint8), 3)
        if encoding in ("rgba8", "bgra8"):
            return (np.dtype(np.uint8), 4)
        if encoding == "mono8":
            return (np.dtype(np.uint8), 1)
        if encoding == "mono16":
            return (np.dtype(np.uint16), 1)

        raise ValueError(f"Unsupported image encoding: {encoding}")

    def _ros_image_to_mediapipe_image(self, image_msg: ImageMsg) -> MediapipeImage:
        """
        Create a MediaPipe image from a ROS Image message.

        If the incoming encoding is supported by MediaPipe, a zero-copy NumPy view
        is created. Otherwise, the byte layout is converted for MediaPipe.
        """

        encoding: str = image_msg.encoding.lower()
        dtype, channels = self._get_numpy_encoding_info(encoding)

        # Build a NumPy view on top of the ROS image buffer to avoid copies.
        itemsize: int = np.dtype(dtype).itemsize
        if channels == 1:
            np_view: np.ndarray = np.ndarray(
                shape=(image_msg.height, image_msg.width),
                dtype=dtype,
                buffer=image_msg.data,
                strides=(image_msg.step, itemsize),
            )
        else:
            pixel_stride: int = itemsize * channels
            np_view = np.ndarray(
                shape=(image_msg.height, image_msg.width, channels),
                dtype=dtype,
                buffer=image_msg.data,
                strides=(image_msg.step, pixel_stride, itemsize),
            )

        if encoding in {"rgb8", "rgba8", "bgra8", "mono8", "mono16"}:
            mp_format = self._get_mediapipe_image_format(encoding)
            return MediapipeImage(image_format=mp_format, data=np_view)

        # Convert unsupported byte layouts such as BGR to RGB byte order for
        # MediaPipe's sRGB input format.
        if encoding == "bgr8":
            rgb_image: np.ndarray = cv2.cvtColor(np_view, cv2.COLOR_BGR2RGB)
            return MediapipeImage(
                image_format=MediapipeImageFormat.SRGB, data=rgb_image
            )

        raise ValueError(f"Unsupported image encoding for MediaPipe: {encoding}")

    def _result_callback(
        self,
        result: vision.PoseLandmarkerResult,
        output_image: MediapipeImage,
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

        header: HeaderMsg | None = self._stamp_map.pop(timestamp_ms, None)
        if header is None:
            header = self._make_header_from_timestamp(timestamp_ms)

        self._publish_pose_messages(result, header)
        self._submit_pending_frame_or_mark_idle()
        self._queue_render_job(result, output_image, header)

    def _queue_render_job(
        self,
        result: vision.PoseLandmarkerResult,
        output_image: MediapipeImage,
        header: HeaderMsg,
    ) -> None:
        if not self._publish_pose_image_enabled:
            return

        render_job: RenderJob = RenderJob(
            result=result,
            output_image=output_image,
            header=header,
        )

        with self._render_lock:
            if self._pending_render_job is not None:
                self._dropped_render_jobs += 1
                if self._dropped_render_jobs % 100 == 0:
                    self.get_logger().debug(
                        "Dropped "
                        f"{self._dropped_render_jobs} pending pose render jobs"
                    )
            self._pending_render_job = render_job

    def _render_pending_image(self) -> None:
        with self._render_lock:
            render_job: RenderJob | None = self._pending_render_job
            self._pending_render_job = None

        if render_job is None:
            return

        self._publish_image(
            render_job.result,
            render_job.output_image,
            render_job.header,
        )

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
        landmarks: Any
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
        output_image: MediapipeImage,
        header: HeaderMsg,
    ) -> None:
        # Convert the mediapipe.Image to a NumPy array
        try:
            annotated_image: np.ndarray = cv2.cvtColor(
                output_image.numpy_view(), cv2.COLOR_RGB2BGR
            )
        except Exception as e:
            self.get_logger().error("Failed to convert mediapipe image: " + str(e))
            return

        # Get image dimensions
        h: int
        w: int
        h, w, _ = annotated_image.shape

        for i, landmarks in enumerate(result.pose_landmarks or []):
            draw_pose_landmarks(annotated_image, landmarks)

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
                (255, 255, 0),  # Cyan in BGR
                thickness=3,
            )

        # Convert the annotated image back to a ROS Image message
        try:
            ros_image_msg = self._cv_bridge.cv2_to_imgmsg(
                annotated_image, encoding="bgr8"
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
