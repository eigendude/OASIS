################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from typing import Any, cast

import cv2
import cv_bridge
import mediapipe
import numpy as np
import rclpy.node
from image_transport_py import ImageTransport
from mediapipe.framework.formats import landmark_pb2

from oasis_msgs.msg import PoseLandmarksArray as PoseLandmarksArrayMsg  # type: ignore[attr-defined]


################################################################################
# ROS parameters
################################################################################


# Default node name
NODE_NAME = "pose_landmarker"

# Publishers
POSE_IMAGE_TOPIC = "pose_image"
POSE_LANDMARKS_TOPIC = "pose"


################################################################################
# ROS node
################################################################################


class PoseRendererNode(rclpy.node.Node):
    """
    Subscribe to pose landmarks for multiple zones and render transparent
    overlays (skeleton + boxes) per zone. Publishes on
    /oasis/<zone>/pose_image_<host_id>.
    """

    def __init__(self) -> None:
        super().__init__("pose_renderer")

        # Declare parameters
        self.declare_parameter("host_id", "")  # Host ID
        self.declare_parameter("zone_ids", [""])  # List of zone IDs
        self.declare_parameter("width", 960)  # Overlay width in px
        self.declare_parameter("height", 540)  # Overlay height in px

        # Read parameters
        host_id: str = self.get_parameter("host_id").get_parameter_value().string_value
        zone_ids: list[str] = (
            self.get_parameter("zone_ids").get_parameter_value().string_array_value
        )
        width: int = self.get_parameter("width").get_parameter_value().integer_value
        height: int = self.get_parameter("height").get_parameter_value().integer_value

        # Prepare CV bridge & mediapipe drawing
        mediapipe_module = cast(Any, mediapipe)
        self._cv_bridge: cv_bridge.CvBridge = cv_bridge.CvBridge()
        self._mp_drawing = mediapipe_module.solutions.drawing_utils
        self._mp_styles = mediapipe_module.solutions.drawing_styles
        self._mp_pose = mediapipe_module.solutions.pose

        # ImageTransport for publishing compressed overlays
        self._it: ImageTransport = ImageTransport(
            self.get_name(), image_transport="compressed"
        )

        # Per‑zone publishers and subscriptions
        self._pubs: dict[str, Any] = {}
        for zone_id in zone_ids:
            topic_in: str = f"{zone_id}/{POSE_LANDMARKS_TOPIC}"
            topic_out: str = f"{host_id}/{zone_id}/{POSE_IMAGE_TOPIC}"

            # Subscribe to PoseLandmarksArrayMsg
            self.create_subscription(
                PoseLandmarksArrayMsg,
                topic_in,
                lambda msg, _, z=zone_id, w=width, h=height: self._on_pose(
                    msg, z, w, h
                ),
                1,
            )

            # Advertise overlay
            self._pubs[zone_id] = self._it.advertise(
                self.resolve_topic_name(topic_out), 1
            )

        self.get_logger().info(f"PoseRendererNode initialized for zones: {zone_ids}")

    def stop(self) -> None:
        self.get_logger().info("PoseRendererNode shutting down...")
        self.destroy_node()

    def _on_pose(
        self,
        msg: PoseLandmarksArrayMsg,
        zone: str,
        width: int,
        height: int,
    ) -> None:
        """
        Callback for each PoseLandmarksArrayMsg per zone. Renders a transparent
        overlay and publishes it.
        """
        # 1) Draw everything on a 3‑channel BGR overlay (this will be C‑contiguous)
        overlay: np.ndarray = np.zeros((height, width, 3), dtype=np.uint8)

        for pose in msg.poses:
            # Build a NormalizedLandmarkList
            landmark_list = landmark_pb2.NormalizedLandmarkList()
            landmark_list.landmark.extend(
                [
                    landmark_pb2.NormalizedLandmark(x=lm.x, y=lm.y, z=lm.z)
                    for lm in pose.landmarks
                ]
            )

            # Draw skeleton on overlay
            self._mp_drawing.draw_landmarks(
                overlay,
                landmark_list,
                self._mp_pose.POSE_CONNECTIONS,
                self._mp_styles.get_default_pose_landmarks_style(),
            )

            # Compute and draw bounding box
            xs = [lm.x for lm in pose.landmarks]
            ys = [lm.y for lm in pose.landmarks]
            x0_n, y0_n, x1_n, y1_n = min(xs), min(ys), max(xs), max(ys)
            px0, py0 = int(x0_n * width), int(y0_n * height)
            px1, py1 = int(x1_n * width), int(y1_n * height)

            cv2.rectangle(
                overlay,
                (px0, py0),
                (px1, py1),
                (0, 255, 255),  # Cyan
                thickness=3,
            )

        # 2) Build 4‑channel RGBA canvas by copying overlay into it
        canvas: np.ndarray = np.zeros((height, width, 4), dtype=np.uint8)
        canvas[..., :3] = overlay

        # 3) Any non‑zero pixel in overlay gets full alpha
        mask = np.any(overlay != 0, axis=2)
        canvas[..., 3][mask] = 255

        # 4) Convert to ROS Image and publish
        try:
            overlay_msg = self._cv_bridge.cv2_to_imgmsg(canvas, encoding="rgba8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert overlay to ImageMsg: {e}")
            return

        overlay_msg.header = msg.header
        self._pubs[zone].publish(overlay_msg)
