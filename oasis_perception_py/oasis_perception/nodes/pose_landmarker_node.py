################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import cv_bridge
import mediapipe
import rclpy.node
import sensor_msgs.msg
from builtin_interfaces.msg import Time as TimeMsg
from image_transport_py import ImageTransport
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from std_msgs.msg import Header as HeaderMsg


################################################################################
# ROS parameters
################################################################################


# Default node name
NODE_NAME = "pose_landmarker"

# Subscribers
IMAGE_SUB_TOPIC = "image_raw"

# Publishers
IMAGE_PUB_TOPIC = "pose_landmarks"


################################################################################
# ROS node
################################################################################


class PoseLandmarkerNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.get_logger().info("Pose Landmarker Node Initialized")

        # Map from timestamp_ms → original header stamp
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

        # Initialize MediaPipe pose detector in IMAGE mode for synchronous processing
        self._detector = self.initialize_detector()

        # Setup MediaPipe drawing utilities
        self._mp_drawing = mediapipe.solutions.drawing_utils
        self._mp_drawing_styles = mediapipe.solutions.drawing_styles
        self._mp_pose = mediapipe.solutions.pose

        # Pose detection state
        self._pose_count: int = 0

    def initialize_detector(self):
        # TODO: Adjust model_asset_path as needed
        model_path = "../oasis_perception_py/mediapipe/pose_landmarker.task"
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM,
            num_poses=3,
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

    def _result_callback(self, result, output_image, timestamp_ms):
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

        # If landmarks are detected, draw them on the original image
        if result.pose_landmarks:
            # Log the number of detected poses
            if self._pose_count != len(result.pose_landmarks):
                self._pose_count = len(result.pose_landmarks)
                self.get_logger().info(f"Detected {len(result.pose_landmarks)} poses")

            for landmarks in result.pose_landmarks:
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
        else:
            if self._pose_count > 0:
                self._pose_count = 0
                self.get_logger().info("No poses detected")

        # Convert the annotated image back to a ROS Image message
        ros_image_msg = self._cv_bridge.cv2_to_imgmsg(annotated_image, encoding="rgb8")

        # Lookup the original header
        original_header: HeaderMsg | None = self._stamp_map.pop(timestamp_ms, None)
        if original_header is not None:
            ros_image_msg.header = original_header
        else:
            # If it wasn't stored (unlikely), reconstruct from timestamp_ms
            secs = timestamp_ms // 1000
            nsecs = int((timestamp_ms % 1000) * 1e6)
            ros_image_msg.header.stamp = TimeMsg(sec=secs, nanosec=nsecs)

        # Publish the annotated image
        self._pub.publish(ros_image_msg)

    def stop(self) -> None:
        self.get_logger().info("Pose Landmarker Node Shutting Down")
        self.destroy_node()
