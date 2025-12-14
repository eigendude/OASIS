################################################################################
#
#  Copyright (C) 2022-2025 Garrett Brown
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

# Minimum change in motor duty cycle required to publish an update
MOTOR_EPSILON: float = 0.02


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
        self._peripheral_input_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=PeripheralInputMsg,
                topic=SUBSCRIBE_PERIPHERAL_INPUT,
                callback=self._on_peripheral_input,
                qos_profile=qos_profile,
            )
        )
        self._peripherals_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=PeripheralScanMsg,
                topic=SUBSCRIBE_PERIPHERALS,
                callback=self._on_peripheral_scan,
                qos_profile=qos_profile,
            )
        )

        # Service clients
        self._capture_input_client: rclpy.client.Client = self._node.create_client(
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
                button_name: str = digital_button.name
                pressed: bool = True if digital_button.pressed else False

                if button_name == "a":
                    a_button = pressed
                if button_name == "b":
                    b_button = pressed
                if button_name == "y":
                    y_button = pressed

            for analog_button in peripheral_input_msg.analog_buttons:
                button_name = analog_button.name
                analog_magnitude: float = analog_button.magnitude

                if button_name == "lefttrigger":
                    left_trigger = analog_magnitude
                elif button_name == "righttrigger":
                    right_trigger = analog_magnitude

            # Calculate throttle
            throttle: float = right_trigger - left_trigger

            # Zero throttle if A or B buttons are not pressed
            if not a_button and not b_button:
                throttle = 0.0

            # Toggle hold speed when Y button is pressed
            if self._last_y_button != y_button:
                self._last_y_button = y_button
                if y_button:
                    self._hold_speed = not self._hold_speed

            # Disable hold speed if A is pressed
            if a_button:
                self._hold_speed = False

            # Max throttle if hold speed is enabled
            if self._hold_speed:
                throttle = 1.0

            # Reduce throttle if B button is not pressed
            if not b_button:
                throttle *= 0.85  # Step 12V down

            magnitude: float = abs(throttle)
            reverse: bool = throttle < 0.0

            # Reduce magnitude by a factor to limit top speed
            magnitude /= 6.0  # Max 2.0V out of 12V supply

            # Update direction
            if self._reverse != reverse:
                self._reverse = reverse

                self._node.get_logger().debug(
                    f"Direction: {'backward' if reverse else 'forward'}"
                )

                self._station_manager.set_motor_direction(reverse)

            # Snap magnitude to 0 if nearby
            target_magnitude: float = magnitude
            if target_magnitude < MOTOR_EPSILON:
                target_magnitude = 0.0

            # Cull messages if possible
            send_pwm: bool = False
            if target_magnitude == 0.0 and self._magnitude != 0.0:
                send_pwm = True
            elif abs(target_magnitude - self._magnitude) >= MOTOR_EPSILON:
                send_pwm = True

            # Update magnitude
            self._magnitude = target_magnitude

            if send_pwm:
                self._node.get_logger().debug(f"Throttle: {throttle}")

                if self._station_manager is not None:
                    self._station_manager.set_motor_pwm(target_magnitude, self._reverse)

    def _on_peripheral_scan(self, peripheral_scan_msg: PeripheralScanMsg) -> None:
        peripheral: PeripheralInfoMsg

        # Check for newly connected joysticks
        for peripheral in peripheral_scan_msg.peripherals:
            # Translate parameters
            peripheral_address: str = peripheral.address

            # Look for joystick types
            if peripheral.type != PeripheralConstantsMsg.TYPE_JOYSTICK:
                continue

            # Ignore joystick if already open
            if self._joysticks.get(peripheral_address) == CONTROLLER_PROFILE:
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
        self._node.get_logger().debug(
            f"Opening joystick {peripheral_address} of type {controller_profile}"
        )

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
