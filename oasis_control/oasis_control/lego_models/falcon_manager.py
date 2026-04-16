################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a LEGO Millenium Falcon model
#

from functools import partial
from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.task

from oasis_msgs.msg import EffectKind as EffectKindMsg
from oasis_msgs.msg import EffectMode as EffectModeMsg
from oasis_msgs.srv import ConfigureEffect as ConfigureEffectSvc
from oasis_msgs.srv import SetEffect as SetEffectSvc


################################################################################
# Hardware configuration
################################################################################


THRUST_LED_PIN: int = 3  # D3

#
# LED noodle: 5V source, 22.1 Ohm resistor
#
# Thrust LED pin to 540 Ohm resistor, to base of BC337 NPN transistor. Emitter
# goes to ground, collector to the LED noodle.
#


################################################################################
# ROS parameters
################################################################################


# Service clients
CLIENT_CONFIGURE_EFFECT = "configure_effect"
CLIENT_SET_EFFECT = "set_effect"


################################################################################
# Manager
################################################################################


class FalconManager:
    """
    Manager for a LEGO Millenium Falcon model.
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        """
        Initialize resources.
        """
        # Construction parameters
        self._node = node

        # Service clients
        self._configure_effect_client: rclpy.client.Client = self._node.create_client(
            srv_type=ConfigureEffectSvc, srv_name=CLIENT_CONFIGURE_EFFECT
        )
        self._set_effect_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetEffectSvc, srv_name=CLIENT_SET_EFFECT
        )
        self._last_thruster_mode: Optional[int] = None

    def initialize(self) -> bool:
        self._node.get_logger().debug("Waiting for falcon services")
        self._node.get_logger().debug("  - Waiting for configure_effect...")
        self._configure_effect_client.wait_for_service()
        self._node.get_logger().debug("  - Waiting for set_effect...")
        self._set_effect_client.wait_for_service()

        self._node.get_logger().debug("Starting falcon configuration")

        self._node.get_logger().debug(
            f"Attaching LED thruster effect on D{THRUST_LED_PIN}"
        )
        if not self._configure_thrust_led_effect():
            return False

        self._node.get_logger().debug("Setting LED thruster to idle")
        if not self._set_thrust_led_effect(0.0):
            return False

        self._node.get_logger().info("Station manager initialized successfully")

        return True

    def set_thrust_led_effect(self, duty_magnitude: float) -> None:
        """Set runtime mode for the MCU-managed Falcon thrust LED effect."""
        if not self._set_thrust_led_effect(duty_magnitude):
            self._node.get_logger().warning("Failed to set Falcon LED thruster effect")

    def _configure_thrust_led_effect(self) -> bool:
        configure_req: ConfigureEffectSvc.Request = ConfigureEffectSvc.Request()
        configure_req.effect_kind = EffectKindMsg.LED_THRUSTER
        configure_req.instance_id = 0
        configure_req.analog_pins = []
        configure_req.digital_pins = []
        configure_req.pwm_pins = [THRUST_LED_PIN]
        configure_req.config_values = []

        future: rclpy.task.Future = self._configure_effect_client.call_async(
            configure_req
        )

        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._node.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

    def _set_thrust_led_effect(self, target_magnitude: float) -> bool:
        clamped_magnitude: float = max(0.0, min(target_magnitude, 1.0))

        mode: int = (
            EffectModeMsg.LED_THRUSTER_MOVING
            if clamped_magnitude > 0.0
            else EffectModeMsg.LED_THRUSTER_IDLE
        )

        if self._last_thruster_mode == mode:
            return True

        mode_name: str = "idle" if mode == EffectModeMsg.LED_THRUSTER_IDLE else "moving"
        self._node.get_logger().debug(
            f"Setting Falcon LED thruster mode to {mode_name}"
        )
        self._last_thruster_mode = mode

        set_effect_req: SetEffectSvc.Request = SetEffectSvc.Request()
        set_effect_req.effect_kind = EffectKindMsg.LED_THRUSTER
        set_effect_req.instance_id = 0
        set_effect_req.mode = mode
        set_effect_req.values = []

        try:
            future: rclpy.task.Future = self._set_effect_client.call_async(
                set_effect_req
            )
        except Exception as ex:
            self._node.get_logger().error(f"Exception while calling service: {ex}")
            self._last_thruster_mode = None
            return False

        future.add_done_callback(
            partial(self._on_set_thrust_led_effect_complete, mode=mode)
        )
        return True

    def _on_set_thrust_led_effect_complete(
        self, future: rclpy.task.Future, mode: int
    ) -> None:
        try:
            response: Optional[SetEffectSvc.Response] = future.result()
        except Exception as ex:
            self._node.get_logger().error(f"Exception while calling service: {ex}")
            if self._last_thruster_mode == mode:
                self._last_thruster_mode = None
            return

        if response is None:
            self._node.get_logger().error(
                "Set effect call completed with no response for Falcon LED thruster"
            )
            if self._last_thruster_mode == mode:
                self._last_thruster_mode = None
