################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Manager for a station helipad guidance system."""

from __future__ import annotations

import rclpy.client
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import rclpy.task

from oasis_control.helipad.helipad_controller import HelipadController
from oasis_drivers.ros.ros_translator import RosTranslator
from oasis_drivers.telemetrix.telemetrix_types import AnalogMode
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import PWMWriteCommand as PWMWriteCommandMsg
from oasis_msgs.srv import SetAnalogMode as SetAnalogModeSvc
from oasis_msgs.srv import SetDigitalMode as SetDigitalModeSvc


################################################################################
# Timing parameters
################################################################################


ANIMATION_TIMER_PERIOD_SECS: float = 0.1


################################################################################
# ROS parameters
################################################################################


# Subscribers
SUBSCRIBE_ANALOG_READING = "analog_reading"

# Service clients
CLIENT_SET_ANALOG_MODE = "set_analog_mode"
CLIENT_SET_DIGITAL_MODE = "set_digital_mode"

# Command publishers
PUBLISH_PWM_CMD = "pwm_write_cmd"


################################################################################
# Manager
################################################################################


class HelipadManager:
    """ROS glue for helipad sensor input and LED guidance output."""

    def __init__(
        self,
        node: rclpy.node.Node,
        ir_pin: int,
        led_pair_a_pin: int,
        led_pair_b_pin: int,
    ) -> None:
        self._node: rclpy.node.Node = node
        self._ir_pin: int = ir_pin
        self._led_pair_a_pin: int = led_pair_a_pin
        self._led_pair_b_pin: int = led_pair_b_pin
        self._last_pwm_duty_cycle_by_pin: dict[int, float] = {}
        self._sensor_voltage: float = 0.0
        self._controller: HelipadController = HelipadController()
        self._animation_timer: rclpy.node.Timer | None = None

        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )
        cmd_qos: rclpy.qos.QoSProfile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
        )

        self._pwm_cmd_pub: rclpy.publisher.Publisher = self._node.create_publisher(
            msg_type=PWMWriteCommandMsg,
            topic=PUBLISH_PWM_CMD,
            qos_profile=cmd_qos,
        )
        self._analog_reading_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=AnalogReadingMsg,
                topic=SUBSCRIBE_ANALOG_READING,
                callback=self._on_analog_reading,
                qos_profile=qos_profile,
            )
        )
        self._set_analog_mode_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetAnalogModeSvc,
            srv_name=CLIENT_SET_ANALOG_MODE,
        )
        self._set_digital_mode_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetDigitalModeSvc,
            srv_name=CLIENT_SET_DIGITAL_MODE,
        )

    def initialize(self) -> bool:
        self._node.get_logger().debug("Waiting for helipad services")
        self._node.get_logger().debug("  - Waiting for set_analog_mode...")
        self._set_analog_mode_client.wait_for_service()
        self._node.get_logger().debug("  - Waiting for set_digital_mode...")
        self._set_digital_mode_client.wait_for_service()

        self._node.get_logger().debug("Starting helipad configuration")

        self._node.get_logger().debug(f"Enabling helipad IR sensor on A{self._ir_pin}")
        if not self._set_analog_mode(self._ir_pin, AnalogMode.INPUT):
            return False

        self._node.get_logger().debug(
            f"Enabling helipad LED pair A PWM on D{self._led_pair_a_pin}"
        )
        if not self._set_digital_mode(self._led_pair_a_pin, DigitalMode.PWM):
            return False

        self._node.get_logger().debug(
            f"Enabling helipad LED pair B PWM on D{self._led_pair_b_pin}"
        )
        if not self._set_digital_mode(self._led_pair_b_pin, DigitalMode.PWM):
            return False

        self._publish_pwm(self._led_pair_a_pin, 0.0)
        self._publish_pwm(self._led_pair_b_pin, 0.0)
        self._animation_timer = self._node.create_timer(
            timer_period_sec=ANIMATION_TIMER_PERIOD_SECS,
            callback=self._on_animation_timer,
        )

        self._node.get_logger().info("Helipad manager initialized successfully")

        return True

    def _set_analog_mode(self, analog_pin: int, analog_mode: AnalogMode) -> bool:
        analog_mode_req: SetAnalogModeSvc.Request = SetAnalogModeSvc.Request()
        analog_mode_req.analog_pin = analog_pin
        analog_mode_req.analog_mode = RosTranslator.analog_mode_to_ros(analog_mode)

        future: rclpy.task.Future = self._set_analog_mode_client.call_async(
            analog_mode_req
        )

        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._node.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

    def _set_digital_mode(self, digital_pin: int, digital_mode: DigitalMode) -> bool:
        digital_mode_req: SetDigitalModeSvc.Request = SetDigitalModeSvc.Request()
        digital_mode_req.digital_pin = digital_pin
        digital_mode_req.digital_mode = RosTranslator.digital_mode_to_ros(digital_mode)

        future: rclpy.task.Future = self._set_digital_mode_client.call_async(
            digital_mode_req
        )

        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._node.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

    def _on_analog_reading(self, analog_reading_msg: AnalogReadingMsg) -> None:
        if analog_reading_msg.analog_pin != self._ir_pin:
            return

        analog_value: float = analog_reading_msg.analog_value
        if not 0.0 <= analog_value <= 1.0:
            self._node.get_logger().warning(
                f"Analog reading {analog_value:.2f} for pin {self._ir_pin} out of "
                "range, clamping to [0.0, 1.0]"
            )

        normalized_value: float = max(0.0, min(analog_value, 1.0))
        self._sensor_voltage = normalized_value * analog_reading_msg.reference_voltage

    def _on_animation_timer(self) -> None:
        timestamp_sec: float = self._node.get_clock().now().nanoseconds / 1e9
        pair_a_duty, pair_b_duty = self._controller.update(
            timestamp_sec,
            self._sensor_voltage,
        )

        self._publish_pwm(self._led_pair_a_pin, pair_a_duty)
        self._publish_pwm(self._led_pair_b_pin, pair_b_duty)

    def _publish_pwm(self, digital_pin: int, duty_cycle: float) -> None:
        last_duty_cycle: float | None = self._last_pwm_duty_cycle_by_pin.get(
            digital_pin
        )
        if last_duty_cycle == duty_cycle:
            return

        pwm_cmd: PWMWriteCommandMsg = PWMWriteCommandMsg()
        pwm_cmd.digital_pin = digital_pin
        pwm_cmd.duty_cycle = duty_cycle
        self._pwm_cmd_pub.publish(pwm_cmd)
        self._last_pwm_duty_cycle_by_pin[digital_pin] = duty_cycle
