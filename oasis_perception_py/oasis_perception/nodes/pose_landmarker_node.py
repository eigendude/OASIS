################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import os

import cv2
import cv_bridge
import mediapipe
import numpy as np
import rclpy.node
from ament_index_python import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from image_transport_py import ImageTransport
from mediapipe import Image as MediapipeImage  # type: ignore[attr-defined]
from mediapipe import ImageFormat as MediapipeImageFormat  # type: ignore[attr-defined]
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from oasis_perception.utils.bounding_box_smoother import BoundingBoxSmoother
from rclpy.logging import LoggingSeverity
from rclpy.qos import qos_profile_default
from sensor_msgs.msg import CameraInfo as CameraInfoMsg
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

        image_transport_param = (
            self.get_parameter(IMAGE_TRANSPORT_PARAM).get_parameter_value().string_value
        )
        image_transport = image_transport_param or IMAGE_TRANSPORT_DEFAULT

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

        # Create an ImageTransport object using this node's name and the
        # configured transport
        self._image_transport: ImageTransport = ImageTransport(
            self.get_name(), image_transport=image_transport
        )

        # Subscribers
        self._camera_subscription = self._image_transport.subscribe_camera(
            base_topic=self.resolve_topic_name(IMAGE_SUB_TOPIC),
            queue_size=1,
            callback=self._camera_callback,
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

    def stop(self) -> None:
        self.get_logger().info("Pose landmarker node shutting down")
        self.destroy_node()

    def _camera_callback(self, image_msg: ImageMsg, camera_info: CameraInfoMsg) -> None:
        if not image_msg.data:
            self.get_logger().error("Received empty image message")
            return

        # Grab the header stamp
        header_stamp = image_msg.header.stamp
        if header_stamp.sec != 0 or header_stamp.nanosec != 0:
            # Use camera driver's timestamp
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
        self._stamp_map[timestamp_ms] = image_msg.header

        try:
            mp_image = self._ros_image_to_mediapipe_image(image_msg)
        except Exception as e:
            self.get_logger().error("Image conversion failed: " + str(e))
            return

        # Submit the frame for asynchronous processing
        self._detector.detect_async(mp_image, timestamp_ms)

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
        is created. Otherwise, the image is converted to SRGB using OpenCV.
        """

        encoding: str = image_msg.encoding.lower()
        dtype, channels = self._get_numpy_encoding_info(encoding)

        # Build a NumPy view on top of the ROS image buffer to avoid copies.
        itemsize: int = np.dtype(dtype).itemsize
        if channels == 1:
            np_view = np.ndarray(
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

        # Convert unsupported encodings (e.g., BGR) to SRGB for MediaPipe.
        if encoding == "bgr8":
            rgb_image = cv2.cvtColor(np_view, cv2.COLOR_BGR2RGB)
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
