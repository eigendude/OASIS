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

import enum
import math
from functools import partial

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription
import rclpy.task
from builtin_interfaces.msg import Time as TimeMsg

from oasis_drivers.ros.ros_translator import RosTranslator
from oasis_drivers.telemetrix.telemetrix_types import AnalogMode
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import EffectKind as EffectKindMsg
from oasis_msgs.msg import EffectMode as EffectModeMsg
from oasis_msgs.srv import ConfigureEffect as ConfigureEffectSvc
from oasis_msgs.srv import SetAnalogMode as SetAnalogModeSvc
from oasis_msgs.srv import SetDigitalMode as SetDigitalModeSvc
from oasis_msgs.srv import SetEffect as SetEffectSvc


################################################################################
# ROS parameters
################################################################################


# Subscribers
SUBSCRIBE_ANALOG_READING = "analog_reading"

# Service clients
CLIENT_CONFIGURE_EFFECT = "configure_effect"
CLIENT_SET_ANALOG_MODE = "set_analog_mode"
CLIENT_SET_DIGITAL_MODE = "set_digital_mode"
CLIENT_SET_EFFECT = "set_effect"


################################################################################
# Helipad control
################################################################################


LAND_THRESHOLD_VOLTS: float = 4.6
LAND_DEBOUNCE_SECS: float = 0.2


class HelipadMode(enum.IntEnum):
    """High-level helipad guidance modes."""

    DISABLED = 0
    GUIDANCE = 1
    LANDED = 2


class HelipadModeTracker:
    """Track helipad guidance mode from reflectance sensor voltage."""

    def __init__(
        self,
        land_threshold_volts: float = LAND_THRESHOLD_VOLTS,
        debounce_secs: float = LAND_DEBOUNCE_SECS,
    ) -> None:
        if not math.isfinite(land_threshold_volts):
            raise ValueError("land_threshold_volts must be finite")
        if not math.isfinite(debounce_secs) or debounce_secs < 0.0:
            raise ValueError("debounce_secs must be finite and non-negative")

        self._land_threshold_volts: float = land_threshold_volts
        self._debounce_secs: float = debounce_secs
        self._landed: bool = False
        self._candidate_landed: bool = False
        self._candidate_since_sec: float | None = None
        self._last_timestamp_sec: float | None = None

    @property
    def landed(self) -> bool:
        """Return whether the helicopter is currently considered landed."""

        return self._landed

    def update_mode(self, timestamp_sec: float, sensor_voltage: float) -> HelipadMode:
        """Advance the tracker and return the high-level guidance mode."""

        current_timestamp_sec: float = self._normalize_timestamp(timestamp_sec)
        self._update_landing_state(current_timestamp_sec, sensor_voltage)

        if self._landed:
            return HelipadMode.LANDED

        return HelipadMode.GUIDANCE

    def _normalize_timestamp(self, timestamp_sec: float) -> float:
        if not math.isfinite(timestamp_sec):
            if self._last_timestamp_sec is not None:
                return self._last_timestamp_sec
            return 0.0

        current_timestamp_sec: float = timestamp_sec
        if self._last_timestamp_sec is not None:
            current_timestamp_sec = max(current_timestamp_sec, self._last_timestamp_sec)

        self._last_timestamp_sec = current_timestamp_sec
        return current_timestamp_sec

    def _update_landing_state(
        self, timestamp_sec: float, sensor_voltage: float
    ) -> None:
        sensed_voltage: float = sensor_voltage if math.isfinite(sensor_voltage) else 0.0
        observed_landed: bool = sensed_voltage <= self._land_threshold_volts

        if observed_landed == self._landed:
            self._candidate_landed = observed_landed
            self._candidate_since_sec = None
            return

        if (
            self._candidate_since_sec is None
            or self._candidate_landed != observed_landed
        ):
            self._candidate_landed = observed_landed
            self._candidate_since_sec = timestamp_sec
            return

        if timestamp_sec - self._candidate_since_sec < self._debounce_secs:
            return

        self._landed = observed_landed
        self._candidate_since_sec = None


################################################################################
# Manager
################################################################################


class HelipadManager:
    """ROS glue for helipad sensor input and high-level LED state changes."""

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
        self._sensor_voltage: float = 0.0
        self._mode_tracker: HelipadModeTracker = HelipadModeTracker()
        self._mode: HelipadMode = HelipadMode.DISABLED
        self._initializing: bool = False
        self._mode_request_future: rclpy.task.Future | None = None
        self._pending_mode: HelipadMode | None = None

        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        self._analog_reading_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=AnalogReadingMsg,
                topic=SUBSCRIBE_ANALOG_READING,
                callback=self._on_analog_reading,
                qos_profile=qos_profile,
            )
        )
        self._configure_effect_client: rclpy.client.Client = self._node.create_client(
            srv_type=ConfigureEffectSvc,
            srv_name=CLIENT_CONFIGURE_EFFECT,
        )
        self._set_analog_mode_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetAnalogModeSvc,
            srv_name=CLIENT_SET_ANALOG_MODE,
        )
        self._set_digital_mode_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetDigitalModeSvc,
            srv_name=CLIENT_SET_DIGITAL_MODE,
        )
        self._set_effect_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetEffectSvc,
            srv_name=CLIENT_SET_EFFECT,
        )

    def initialize(self) -> bool:
        self._initializing = True

        self._node.get_logger().debug("Waiting for helipad services")
        self._node.get_logger().debug("  - Waiting for configure_effect...")
        self._configure_effect_client.wait_for_service()
        self._node.get_logger().debug("  - Waiting for set_analog_mode...")
        self._set_analog_mode_client.wait_for_service()
        self._node.get_logger().debug("  - Waiting for set_digital_mode...")
        self._set_digital_mode_client.wait_for_service()
        self._node.get_logger().debug("  - Waiting for set_effect...")
        self._set_effect_client.wait_for_service()

        try:
            self._node.get_logger().debug("Starting helipad configuration")

            self._node.get_logger().debug(
                f"Enabling helipad IR sensor on A{self._ir_pin}"
            )
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

            self._node.get_logger().debug("Attaching helipad feature on the MCU")
            if not self._attach_helipad():
                return False

            if not self._set_mode(HelipadMode.DISABLED):
                return False

            self._node.get_logger().info("Helipad manager initialized successfully")

            return True
        finally:
            self._initializing = False

    def _attach_helipad(self) -> bool:
        configure_req: ConfigureEffectSvc.Request = ConfigureEffectSvc.Request()
        configure_req.effect_kind = EffectKindMsg.HELIPAD
        configure_req.instance_id = 0
        configure_req.analog_pins = [self._ir_pin]
        configure_req.pwm_pins = [self._led_pair_a_pin, self._led_pair_b_pin]

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

    def _set_mode(self, mode: HelipadMode) -> bool:
        if self._mode == mode:
            return True

        set_effect_req: SetEffectSvc.Request = SetEffectSvc.Request()
        set_effect_req.effect_kind = EffectKindMsg.HELIPAD
        set_effect_req.instance_id = 0
        set_effect_req.mode = self._mode_to_ros(mode)

        future: rclpy.task.Future = self._set_effect_client.call_async(set_effect_req)

        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._node.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        self._mode = mode
        return True

    def _set_mode_async(self, mode: HelipadMode) -> None:
        if self._mode == mode and self._mode_request_future is None:
            self._node.get_logger().debug(
                "HELIPAD mode_skip "
                f"mode={mode.name} node_t={self._node_time_sec():.6f}"
            )
            return

        if self._mode_request_future is not None:
            self._pending_mode = mode
            self._node.get_logger().debug(
                "HELIPAD mode_pending "
                f"mode={mode.name} current={self._mode.name} "
                f"node_t={self._node_time_sec():.6f}"
            )
            return

        self._dispatch_mode_request(mode)

    def _dispatch_mode_request(self, mode: HelipadMode) -> None:
        self._node.get_logger().debug(
            "HELIPAD mode_dispatch "
            f"mode={mode.name} node_t={self._node_time_sec():.6f}"
        )

        set_effect_req: SetEffectSvc.Request = SetEffectSvc.Request()
        set_effect_req.effect_kind = EffectKindMsg.HELIPAD
        set_effect_req.instance_id = 0
        set_effect_req.mode = self._mode_to_ros(mode)

        future: rclpy.task.Future = self._set_effect_client.call_async(set_effect_req)
        future.add_done_callback(partial(self._on_set_mode_done, mode=mode))

        self._mode_request_future = future

    def _on_set_mode_done(self, future: rclpy.task.Future, mode: HelipadMode) -> None:
        self._mode_request_future = None

        succeeded: bool = future.result() is not None
        self._node.get_logger().debug(
            "HELIPAD mode_done "
            f"mode={mode.name} success={succeeded} "
            f"node_t={self._node_time_sec():.6f}"
        )

        if not succeeded:
            self._node.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
        else:
            self._mode = mode

        next_mode: HelipadMode | None = self._pending_mode
        self._pending_mode = None
        will_dispatch: bool = next_mode is not None and next_mode != self._mode
        next_mode_name: str = next_mode.name if next_mode is not None else "None"
        self._node.get_logger().debug(
            "HELIPAD mode_done_pending "
            f"pending={next_mode_name} will_dispatch={will_dispatch} "
            f"node_t={self._node_time_sec():.6f}"
        )

        if will_dispatch and next_mode is not None:
            self._dispatch_mode_request(next_mode)

    def _on_analog_reading(self, analog_reading_msg: AnalogReadingMsg) -> None:
        if analog_reading_msg.analog_pin != self._ir_pin:
            return

        analog_pin: int = analog_reading_msg.analog_pin
        analog_value: float = analog_reading_msg.analog_value
        if not 0.0 <= analog_value <= 1.0:
            self._node.get_logger().warning(
                f"Analog reading {analog_value:.2f} for pin {self._ir_pin} out of "
                "range, clamping to [0.0, 1.0]"
            )

        normalized_value: float = max(0.0, min(analog_value, 1.0))
        self._sensor_voltage = normalized_value * analog_reading_msg.reference_voltage
        message_timestamp_sec: float = self._time_msg_to_sec(
            analog_reading_msg.header.stamp
        )
        node_timestamp_sec: float = self._node_time_sec()
        self._node.get_logger().debug(
            "HELIPAD analog "
            f"pin=A{analog_pin} raw_norm={analog_value:.4f} "
            f"norm={normalized_value:.4f} voltage={self._sensor_voltage:.4f} "
            f"msg_t={message_timestamp_sec:.6f} node_t={node_timestamp_sec:.6f}"
        )

        if self._initializing:
            return

        mode: HelipadMode = self._mode_tracker.update_mode(
            node_timestamp_sec,
            self._sensor_voltage,
        )
        self._node.get_logger().debug(
            "HELIPAD mode_decision "
            f"mode={mode.name} voltage={self._sensor_voltage:.4f} "
            f"landed={self._mode_tracker.landed} node_t={node_timestamp_sec:.6f}"
        )
        self._set_mode_async(mode)

    def _node_time_sec(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _time_msg_to_sec(stamp: TimeMsg) -> float:
        sec: int = int(stamp.sec)
        nanosec: int = int(stamp.nanosec)
        return float(sec) + float(nanosec) * 1.0e-9

    @staticmethod
    def _mode_to_ros(mode: HelipadMode) -> int:
        mode_map: dict[HelipadMode, int] = {
            HelipadMode.DISABLED: EffectModeMsg.HELIPAD_DISABLED,
            HelipadMode.GUIDANCE: EffectModeMsg.HELIPAD_GUIDANCE,
            HelipadMode.LANDED: EffectModeMsg.HELIPAD_LANDED,
        }
        return mode_map[mode]
