################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Manage the Kodi train HUD from controller input."""

from __future__ import annotations

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from std_msgs.msg import Bool as BoolMsg

from oasis_control.managers.train_hud_manager import TrainHudManager
from oasis_control.ros.qos_profiles import reliable_state_qos
from oasis_msgs.msg import PeripheralInput as PeripheralInputMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "train_hud_manager"

INPUT_TOPIC: str = "input"
SHOW_SLAM_TOPIC: str = "show_slam"

UP_BUTTON_NAME: str = "up"


################################################################################
# ROS node
################################################################################


class TrainHudNode(rclpy.node.Node):
    """Toggle Kodi's SLAM view on distinct controller up-button presses."""

    def __init__(self) -> None:
        """Initialize subscriptions, state, and the visibility publisher."""

        super().__init__(NODE_NAME)

        self._train_hud_manager: TrainHudManager = TrainHudManager()

        state_qos_profile: rclpy.qos.QoSProfile = reliable_state_qos()
        input_qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        self._show_slam_pub: rclpy.publisher.Publisher[BoolMsg] = self.create_publisher(
            msg_type=BoolMsg,
            topic=SHOW_SLAM_TOPIC,
            qos_profile=state_qos_profile,
        )
        self._input_sub: rclpy.subscription.Subscription[PeripheralInputMsg] = (
            self.create_subscription(
                msg_type=PeripheralInputMsg,
                topic=INPUT_TOPIC,
                callback=self._on_input,
                qos_profile=input_qos_profile,
            )
        )

        self._publish_state(self._train_hud_manager.show_slam)

    def _on_input(self, input_msg: PeripheralInputMsg) -> None:
        for digital_button in input_msg.digital_buttons:
            if digital_button.name != UP_BUTTON_NAME:
                continue

            new_state: bool | None = self._train_hud_manager.update_up_button(
                bool(digital_button.pressed)
            )
            if new_state is not None:
                self.get_logger().debug(
                    f"SLAM view {'shown' if new_state else 'hidden'}"
                )
                self._publish_state(new_state)
            return

    def _publish_state(self, show_slam: bool) -> None:
        show_slam_msg: BoolMsg = BoolMsg()
        show_slam_msg.data = show_slam
        self._show_slam_pub.publish(show_slam_msg)
