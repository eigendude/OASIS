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

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription
import rclpy.task

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

# Unitless duty-cycle cap for a full safe train command, derived from the
# legacy full-right-trigger, no-B command
MAX_SAFE_MOTOR_DUTY_CYCLE: float = 0.142

# Unitless B-button command boost, derived from the legacy 1.176
# boosted/unboosted command ratio
B_BUTTON_TRAIN_COMMAND_BOOST: float = 0.176  # 17.6% boost when B is pressed


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

    def __init__(self, node: rclpy.node.Node, station_manager: StationManager) -> None:
        """
        Initialize resources.
        """
        # Construction parameters
        self._node = node
        self._station_manager: StationManager = station_manager

        # Initialize peripheral state
        self._joysticks: Dict[str, str] = {}

        # Initialize input state
        self._magnitude: float = 0.0
        self._reverse: bool = False  # True if magnitude is in reverse (high DIR pin)
        self._last_y_button: bool = False  # Set to the last value of the Y button
        self._hold_speed: bool = (
            False  # True to hold a steady speed, toggled with Y button
        )

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

    def _on_peripheral_input(self, peripheral_input_msg: PeripheralInputMsg) -> None:
        # Translate parameters
        peripheral_address: str = peripheral_input_msg.address

        if peripheral_address in self._joysticks:
            # Look for button states
            a_button: bool = False
            b_button: bool = False
            y_button: bool = False
            left_trigger: float = 0.0
            right_trigger: float = 0.0

            for digital_button in peripheral_input_msg.digital_buttons:
                digital_button_name: str = digital_button.name
                pressed: bool = True if digital_button.pressed else False

                if digital_button_name == "a":
                    a_button = pressed
                if digital_button_name == "b":
                    b_button = pressed
                if digital_button_name == "y":
                    y_button = pressed

            for analog_button in peripheral_input_msg.analog_buttons:
                analog_button_name: str = analog_button.name
                analog_magnitude: float = analog_button.magnitude

                if analog_button_name == "lefttrigger":
                    left_trigger = analog_magnitude
                elif analog_button_name == "righttrigger":
                    right_trigger = analog_magnitude

            # Raw controller trigger command, normalized to [-1, 1]
            trigger_command: float = right_trigger - left_trigger

            # Zero train command if A or B buttons are not pressed
            train_command: float = trigger_command
            if not a_button and not b_button:
                train_command = 0.0

            # Toggle hold speed when Y button is pressed
            if self._last_y_button != y_button:
                self._last_y_button = y_button
                if y_button:
                    self._hold_speed = not self._hold_speed

            # Disable hold speed if A is pressed
            if a_button:
                self._hold_speed = False

            # Full forward train command if hold speed is enabled
            if self._hold_speed:
                train_command = 1.0

            # B retains the legacy boost behavior within the safe duty cap
            safe_train_command: float = train_command
            if b_button:
                safe_train_command *= 1.0 + B_BUTTON_TRAIN_COMMAND_BOOST

            safe_train_command = max(-1.0, min(safe_train_command, 1.0))

            reverse: bool = safe_train_command < 0.0
            motor_duty_command: float = (
                abs(safe_train_command) * MAX_SAFE_MOTOR_DUTY_CYCLE
            )

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

            # Update magnitude
            self._magnitude = motor_duty_command

            self._node.get_logger().debug(
                "Train command: "
                f"trigger={trigger_command:.3f} "
                f"safe={safe_train_command:.3f} "
                f"duty={motor_duty_command:.3f}"
            )

            # HUD motor voltage is measured separately from ADC telemetry;
            # this value is only the H-bridge duty command
            self._station_manager.set_motor_pwm(motor_duty_command, self._reverse)

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
