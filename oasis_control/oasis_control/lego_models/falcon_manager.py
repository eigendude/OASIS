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
# anager for a LEGO Millenium Falcon model
#

import rclpy.client
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.task

from oasis_drivers.ros.ros_translator import RosTranslator
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode
from oasis_msgs.msg import PWMWriteCommand as PWMWriteCommandMsg
from oasis_msgs.srv import SetDigitalMode as SetDigitalModeSvc


################################################################################
# Hardware configuration
################################################################################


THRUST_LED_PIN: int = 3  # D3

#
# LED noodle: 5V source, 22.1 Ohm resistor
#
# Thrust LED ping to 540 Ohm resistor, to base of BC337 NPN transistor. Emitter
# goes to ground, collector to the LED noodle.
#

################################################################################
# ROS parameters
################################################################################


# Service clients
CLIENT_SET_DIGITAL_MODE = "set_digital_mode"

# Command publishers
PUBLISH_PWM_CMD = "pwm_write_cmd"


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

        # Publishers
        cmd_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
        )
        self._pwm_cmd_pub: rclpy.publisher.Publisher = self._node.create_publisher(
            msg_type=PWMWriteCommandMsg,
            topic=PUBLISH_PWM_CMD,
            qos_profile=cmd_qos,
        )

        # Service clients
        self._set_digital_mode_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetDigitalModeSvc, srv_name=CLIENT_SET_DIGITAL_MODE
        )

    def initialize(self) -> bool:
        self._node.get_logger().debug("Waiting for falcon services")
        self._node.get_logger().debug("  - Waiting for set_digital_mode...")
        self._set_digital_mode_client.wait_for_service()

        self._node.get_logger().debug("Starting falcon configuration")

        # Initialize thrust LED
        self._node.get_logger().debug(f"Enabling thrust LED on D{THRUST_LED_PIN}")
        if not self._set_digital_mode(THRUST_LED_PIN, DigitalMode.PWM):
            return False

        # Turn thrust LED fully on
        self._node.get_logger().debug("Turning thrust LED on")
        self.set_thrust_led_pwm(1.0)

        self._node.get_logger().info("Station manager initialized successfully")

        return True

    def set_thrust_led_pwm(self, target_magnitude: float) -> None:
        """Publish command for motor PWM"""
        pwm_cmd = PWMWriteCommandMsg()
        pwm_cmd.digital_pin = THRUST_LED_PIN
        pwm_cmd.duty_cycle = target_magnitude

        self._pwm_cmd_pub.publish(pwm_cmd)

    def _set_digital_mode(self, digital_pin: int, digital_mode: DigitalMode) -> bool:
        # Create message
        motor_pwm_svc = SetDigitalModeSvc.Request()
        motor_pwm_svc.digital_pin = digital_pin
        motor_pwm_svc.digital_mode = RosTranslator.digital_mode_to_ros(digital_mode)

        # Call service
        future: rclpy.task.Future = self._set_digital_mode_client.call_async(
            motor_pwm_svc
        )

        # Wait for result
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._node.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True
