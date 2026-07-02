################################################################################
#
#  Copyright (C) 2022-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a LEGO train station's input
#

from typing import Dict
from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription
import rclpy.task

from oasis_control.input.checkerboard_slowdown import CheckerboardCruiseSlowdown
from oasis_control.input.park_mode import TrainParkMode
from oasis_control.lego_models.station_manager import StationManager
from oasis_msgs.msg import PeripheralConstants as PeripheralConstantsMsg
from oasis_msgs.msg import PeripheralInfo as PeripheralInfoMsg
from oasis_msgs.msg import PeripheralInput as PeripheralInputMsg
from oasis_msgs.msg import PeripheralScan as PeripheralScanMsg
from oasis_msgs.srv import CaptureInput as CaptureInputSvc


################################################################################
# Input parameters
################################################################################


# The Kodi controller profile that peripheral input is translated to
CONTROLLER_PROFILE = "game.controller.default"

# Motor duty commands below this magnitude are treated as stopped
MOTOR_EPSILON: float = 0.02

# Nominal motor-voltage target measured from the normal full-speed A-button
# command around the track, in volts
NOMINAL_MOTOR_VOLTAGE: float = 6.8

# Maximum motor-voltage target allowed while X is pressed, in volts
MAX_MOTOR_VOLTAGE: float = 8.0

# Unitless duty-cycle cap for the nominal motor-voltage target
MAX_SAFE_MOTOR_DUTY_CYCLE: float = 0.143

# Unitless command cap while X is pressed, derived from the boosted voltage
# target divided by the nominal voltage target
MAX_BOOSTED_TRAIN_COMMAND: float = MAX_MOTOR_VOLTAGE / NOMINAL_MOTOR_VOLTAGE

# Unitless duty-cycle delta required before repeating train command debug logs.
# This reports meaningful speed changes while ignoring controller jitter.
TRAIN_COMMAND_DEBUG_DUTY_EPSILON: float = MAX_SAFE_MOTOR_DUTY_CYCLE / 10.0


################################################################################
# ROS parameters
################################################################################


# Subscribers
SUBSCRIBE_PERIPHERAL_INPUT = "input"
SUBSCRIBE_PERIPHERALS = "peripherals"

# Service clients
CLIENT_CAPTURE_INPUT = "capture_input"


################################################################################
# ROS node
################################################################################


class StationInput:
    """
    A ROS node that manages input for a model train station.
    """

    def __init__(
        self,
        node: rclpy.node.Node,
        station_manager: StationManager,
        checkerboard_slowdown: CheckerboardCruiseSlowdown,
        park_mode: TrainParkMode,
    ) -> None:
        """
        Initialize resources.
        """
        # Construction parameters
        self._node = node
        self._station_manager: StationManager = station_manager
        self._checkerboard_slowdown: CheckerboardCruiseSlowdown = checkerboard_slowdown
        self._park_mode: TrainParkMode = park_mode

        # Initialize peripheral state
        self._joysticks: Dict[str, str] = {}

        # Initialize input state
        self._magnitude: float = 0.0
        self._reverse: bool = False  # True if magnitude is in reverse (high DIR pin)
        self._last_y_button: bool = False  # Set to the last value of the Y button
        self._x_button: bool = False
        self._last_x_button: bool = False  # Set to the last logged X button value
        self._last_start_button: bool = False
        self._hold_speed: bool = (
            False  # True to hold a steady speed, toggled with Y button
        )
        self._last_logged_motor_duty_command: Optional[float] = None

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribers
        self._peripheral_input_sub: rclpy.subscription.Subscription[
            PeripheralInputMsg
        ] = self._node.create_subscription(
            msg_type=PeripheralInputMsg,
            topic=SUBSCRIBE_PERIPHERAL_INPUT,
            callback=self._on_peripheral_input,
            qos_profile=qos_profile,
        )
        self._peripherals_sub: rclpy.subscription.Subscription[PeripheralScanMsg] = (
            self._node.create_subscription(
                msg_type=PeripheralScanMsg,
                topic=SUBSCRIBE_PERIPHERALS,
                callback=self._on_peripheral_scan,
                qos_profile=qos_profile,
            )
        )

        # Service clients
        self._capture_input_client: rclpy.client.Client[
            CaptureInputSvc.Request, CaptureInputSvc.Response
        ] = self._node.create_client(
            srv_type=CaptureInputSvc, srv_name=CLIENT_CAPTURE_INPUT
        )

    @property
    def magnitude(self) -> float:
        return self._magnitude

    @property
    def reverse(self) -> bool:
        return self._reverse

    def initialize(self) -> bool:
        return True

    @property
    def hold_speed(self) -> bool:
        return self._hold_speed

    def update_checkerboard_status(
        self,
        checkerboard_visible: bool,
        now_sec: float,
    ) -> bool:
        if self._park_mode.update_checkerboard_status(checkerboard_visible):
            self._stop_train()
            self._node.get_logger().info("Park mode complete; train parked")
            return False

        if self._park_mode.active:
            return False

        interrupted: bool = self._checkerboard_slowdown.update_checkerboard_status(
            checkerboard_visible,
            now_sec,
        )

        if interrupted and not self._hold_speed:
            self.cancel_checkerboard_slowdown()
            return False

        if interrupted:
            self._apply_hold_speed(now_sec)

        return interrupted

    def update_checkerboard_slowdown(self, now_sec: float) -> bool:
        hold_speed_was_active: bool = self._hold_speed

        expired: bool = self._checkerboard_slowdown.consume_slowdown_expiration(now_sec)
        if expired and hold_speed_was_active and self._hold_speed:
            self._apply_hold_speed(now_sec)
        elif expired:
            self._node.get_logger().debug(
                "Checkerboard slowdown expired after cruise ended; "
                "not reapplying hold speed"
            )

        return expired

    def update_autonomous_train_control(self, now_sec: float) -> bool:
        expired: bool = self.update_checkerboard_slowdown(now_sec)

        if self._park_mode.active:
            self._apply_park_mode()
            return expired

        if self._hold_speed:
            self._apply_hold_speed(now_sec)

        return expired

    def consume_checkerboard_slowdown_activation(
        self,
        now_sec: float,
    ) -> bool:
        return self._checkerboard_slowdown.consume_slowdown_activation(now_sec)

    def checkerboard_slowdown_remaining_sec(self, now_sec: float) -> float:
        return self._checkerboard_slowdown.slowdown_remaining_sec(now_sec)

    def consume_checkerboard_waiting_for_clear_activation(
        self,
        now_sec: float,
    ) -> bool:
        return self._checkerboard_slowdown.consume_waiting_for_clear_activation(now_sec)

    def consume_checkerboard_rearmed(
        self,
        now_sec: float,
    ) -> bool:
        return self._checkerboard_slowdown.consume_rearmed(now_sec)

    def cancel_checkerboard_slowdown(self) -> bool:
        return self._checkerboard_slowdown.cancel()

    def _on_peripheral_input(self, peripheral_input_msg: PeripheralInputMsg) -> None:
        # Translate parameters
        peripheral_address: str = peripheral_input_msg.address

        if peripheral_address in self._joysticks:
            # Look for button states
            a_button: bool = False
            b_button: bool = False
            x_button: bool = False
            y_button: bool = False
            start_button: bool = False
            left_trigger: float = 0.0
            right_trigger: float = 0.0

            for digital_button in peripheral_input_msg.digital_buttons:
                digital_button_name: str = digital_button.name
                pressed: bool = True if digital_button.pressed else False

                if digital_button_name == "a":
                    a_button = pressed
                if digital_button_name == "b":
                    b_button = pressed
                if digital_button_name == "x":
                    x_button = pressed
                if digital_button_name == "y":
                    y_button = pressed
                if digital_button_name == "start":
                    start_button = pressed

            for analog_button in peripheral_input_msg.analog_buttons:
                analog_button_name: str = analog_button.name
                analog_magnitude: float = analog_button.magnitude

                if analog_button_name == "lefttrigger":
                    left_trigger = analog_magnitude
                elif analog_button_name == "righttrigger":
                    right_trigger = analog_magnitude

            self._x_button = x_button

            if b_button:
                self._cancel_park_mode()
                self._hold_speed = False
                self.cancel_checkerboard_slowdown()
                self._stop_train()
                return

            # Raw controller trigger command, normalized to [-1, 1]
            trigger_command: float = right_trigger - left_trigger
            manual_train_command_active: bool = abs(trigger_command) >= MOTOR_EPSILON

            # Zero train command if A or X buttons are not pressed
            train_command: float = trigger_command
            if not a_button and not x_button:
                train_command = 0.0

            # Toggle hold speed when Y button is pressed
            hold_speed_before_input: bool = self._hold_speed
            if self._last_y_button != y_button:
                self._last_y_button = y_button
                if y_button:
                    self._hold_speed = not self._hold_speed

            if self._last_start_button != start_button:
                self._last_start_button = start_button
                if start_button:
                    self._activate_park_mode()

            # Disable hold speed if manual train ownership or park mode is active
            if a_button or manual_train_command_active or self._park_mode.active:
                self._hold_speed = False

            if hold_speed_before_input and not self._hold_speed:
                if self.cancel_checkerboard_slowdown():
                    self._node.get_logger().info(
                        "Checkerboard slowdown cancelled because cruise ended"
                    )

            if a_button or y_button:
                self._cancel_park_mode()

            if self._park_mode.active:
                self._hold_speed = False
                self.cancel_checkerboard_slowdown()
                self.update_autonomous_train_control(self._now_sec())
                return

            self._log_boost_change(x_button)

            if self._hold_speed:
                self.update_autonomous_train_control(self._now_sec())
                return

            self._apply_train_command(train_command, boost_enabled=x_button)

    def _apply_hold_speed(self, now_sec: float) -> None:
        if not self._hold_speed:
            return

        train_command: float = self._checkerboard_slowdown.scale_command(
            1.0,
            cruise_active=True,
            now_sec=now_sec,
        )
        self._apply_train_command(train_command, boost_enabled=self._x_button)

    def _apply_park_mode(self) -> None:
        if not self._park_mode.active:
            return

        self._hold_speed = False
        self.cancel_checkerboard_slowdown()
        self._apply_train_command(
            -self._park_mode.command,
            boost_enabled=self._x_button,
        )

    def _apply_train_command(self, command: float, boost_enabled: bool) -> None:
        unboosted_safe_train_command: float = max(-1.0, min(command, 1.0))

        # X scales the command toward the measured maximum voltage target
        safe_train_command: float = unboosted_safe_train_command
        if boost_enabled:
            safe_train_command *= MAX_BOOSTED_TRAIN_COMMAND

        safe_train_command = max(
            -MAX_BOOSTED_TRAIN_COMMAND,
            min(safe_train_command, MAX_BOOSTED_TRAIN_COMMAND),
        )

        reverse: bool = safe_train_command < 0.0
        motor_duty_command: float = abs(safe_train_command) * MAX_SAFE_MOTOR_DUTY_CYCLE

        # Update direction
        if self._reverse != reverse:
            self._reverse = reverse

            self._node.get_logger().debug(
                f"Direction: {'backward' if reverse else 'forward'}"
            )

            self._station_manager.set_motor_direction(reverse)

        # Snap magnitude to 0 if nearby
        if motor_duty_command < MOTOR_EPSILON:
            motor_duty_command = 0.0

        target_motor_voltage: float = safe_train_command * NOMINAL_MOTOR_VOLTAGE
        if motor_duty_command == 0.0:
            target_motor_voltage = 0.0

        # Update magnitude
        self._magnitude = motor_duty_command

        previous_logged_duty: Optional[float] = self._last_logged_motor_duty_command
        should_log_train_command: bool = previous_logged_duty is None
        if previous_logged_duty is not None:
            stopped_changed: bool = (previous_logged_duty == 0.0) != (
                motor_duty_command == 0.0
            )
            duty_changed: bool = (
                abs(motor_duty_command - previous_logged_duty)
                >= TRAIN_COMMAND_DEBUG_DUTY_EPSILON
            )
            should_log_train_command = stopped_changed or duty_changed

        if should_log_train_command:
            self._last_logged_motor_duty_command = motor_duty_command

            self._node.get_logger().debug(
                "Train command: "
                f"v={target_motor_voltage:.2f}V "
                f"cmd={safe_train_command:.3f} "
                f"duty={motor_duty_command:.3f}"
            )

        # HUD motor voltage is measured separately from ADC telemetry;
        # this value is only the H-bridge duty command
        self._station_manager.set_motor_pwm(motor_duty_command, self._reverse)

    def _log_boost_change(self, x_button: bool) -> None:
        if x_button == self._last_x_button:
            return

        self._last_x_button = x_button

        self._node.get_logger().debug(
            "Train boost: "
            f"{'enabled' if x_button else 'disabled'} "
            f"x={MAX_BOOSTED_TRAIN_COMMAND:.3f} "
            f"max={MAX_MOTOR_VOLTAGE:.2f}V"
        )

    def _activate_park_mode(self) -> None:
        if not self._park_mode.enabled:
            self._node.get_logger().info("Park mode disabled")
            return

        self._hold_speed = False
        self.cancel_checkerboard_slowdown()

        if self._park_mode.activate():
            self._node.get_logger().info("Park mode active; reversing train")

    def _cancel_park_mode(self) -> None:
        if self._park_mode.cancel():
            self._node.get_logger().info("Park mode cancelled")

    def _stop_train(self) -> None:
        self._hold_speed = False
        self._magnitude = 0.0
        self._reverse = False
        self._last_logged_motor_duty_command = None

        self._station_manager.set_motor_direction(False)
        self._station_manager.set_motor_pwm(0.0, False)

    def _now_sec(self) -> float:
        now = self._node.get_clock().now()
        nanoseconds: Optional[int] = getattr(now, "nanoseconds", None)
        if nanoseconds is not None:
            return float(nanoseconds) * 1.0e-9

        stamp = now.to_msg()
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _on_peripheral_scan(self, peripheral_scan_msg: PeripheralScanMsg) -> None:
        peripheral: PeripheralInfoMsg

        # Check for newly connected joysticks
        for peripheral in peripheral_scan_msg.peripherals:
            # Translate parameters
            peripheral_address: str = peripheral.address

            # Look for joystick types
            if peripheral.type != PeripheralConstantsMsg.TYPE_JOYSTICK:
                continue

            # Update peripheral state
            self._joysticks[peripheral_address] = CONTROLLER_PROFILE

            # Open the joystick to capture input
            self._open_joystick(peripheral_address, CONTROLLER_PROFILE)

        # Check for disconnected joysticks
        joysticks_copy = self._joysticks.copy()
        for address, profile in joysticks_copy.items():
            found: bool = any(
                peripheral.address == address
                for peripheral in peripheral_scan_msg.peripherals
            )
            if not found:
                self._node.get_logger().debug(
                    f"Closing joystick {address} of type {profile}"
                )
                del self._joysticks[address]

    def _open_joystick(self, peripheral_address: str, controller_profile: str) -> None:
        # Create message
        capture_input_svc = CaptureInputSvc.Request()
        capture_input_svc.capture = 1
        capture_input_svc.peripheral_address = peripheral_address
        capture_input_svc.controller_profile = CONTROLLER_PROFILE

        # Call service
        future: rclpy.task.Future = self._capture_input_client.call_async(
            capture_input_svc
        )

        # Wait for result
        future.result()
