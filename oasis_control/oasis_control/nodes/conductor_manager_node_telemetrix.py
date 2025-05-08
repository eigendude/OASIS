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
# Manager for a LEGO train power station's microcontroller
#

import asyncio
from typing import Dict
from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Header as HeaderMsg

from oasis_control.managers.cpu_fan_manager import CPUFanManager
from oasis_control.managers.mcu_memory_manager_telemetrix import McuMemoryManager
from oasis_control.managers.sampling_manager import SamplingManager
from oasis_drivers.ros.ros_translator import RosTranslator
from oasis_drivers.telemetrix.telemetrix_types import AnalogMode
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import ConductorState as ConductorStateMsg
from oasis_msgs.msg import DigitalReading as DigitalReadingMsg
from oasis_msgs.msg import PeripheralConstants as PeripheralConstantsMsg
from oasis_msgs.msg import PeripheralInfo as PeripheralInfoMsg
from oasis_msgs.msg import PeripheralInput as PeripheralInputMsg
from oasis_msgs.msg import PeripheralScan as PeripheralScanMsg
from oasis_msgs.msg import PowerMode as PowerModeMsg
from oasis_msgs.srv import CaptureInput as CaptureInputSvc
from oasis_msgs.srv import DigitalWrite as DigitalWriteSvc
from oasis_msgs.srv import PowerControl as PowerControlSvc
from oasis_msgs.srv import PWMWrite as PWMWriteSvc
from oasis_msgs.srv import SetAnalogMode as SetAnalogModeSvc
from oasis_msgs.srv import SetDigitalMode as SetDigitalModeSvc


################################################################################
# Hardware configuration
################################################################################


# CPU fan sampling interval, in ms
CPU_FAN_SAMPLING_INTERVAL_MS = 100

# Memory reporting interval, in seconds
REPORT_MCU_MEMORY_PERIOD_SECS: float = 1.0

# Sampling interval, in ms
SAMPLING_INTERVAL_MS = 100

# Pins
VSS_PIN: int = 0  # A0
MOTOR_PWM_PIN: int = 5  # D5
MOTOR_DIR_PIN: int = 4  # D4
MOTOR_FF1_PIN: int = 8  # D8
MOTOR_FF2_PIN: int = 7  # D7
MOTOR_CURRENT_PIN: int = 1  # A1
CPU_FAN_PWM_PIN: int = 9  # D9
CPU_FAN_SPEED_PIN: int = 2  # D2

# Voltage dividers
# R1 is the input-side resistor, R2 is the ground-side resistor
VSS_R1: float = 147.9  # KΩ
VSS_R2: float = 102.5  # KΩ


################################################################################
# Input parameters
################################################################################


# The Kodi controller profile that peripheral input is translated to
CONTROLLER_PROFILE = "game.controller.default"


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = "conductor_manager"

PUBLISH_STATE_PERIOD_SECS = 0.1

# Publisher
PUBLISH_CONDUCTOR_STATE = "conductor_state"

# Subscribers
SUBSCRIBE_ANALOG_READING = "analog_reading"
SUBSCRIBE_CPU_FAN_SPEED = "cpu_fan_speed"
SUBSCRIBE_DIGITAL_READING = "digital_reading"
SUBSCRIBE_MCU_MEMORY = "mcu_memory"
SUBSCRIBE_MCU_STRING = "mcu_string"
SUBSCRIBE_PERIPHERAL_INPUT = "input"
SUBSCRIBE_PERIPHERALS = "peripherals"

# Services
SERVICE_POWER_CONTROL = "power_control"

# Service clients
CLIENT_CAPTURE_INPUT = "capture_input"
CLIENT_DIGITAL_WRITE = "digital_write"
CLIENT_PWM_WRITE = "pwm_write"
CLIENT_REPORT_MCU_MEMORY = "report_mcu_memory"
CLIENT_SET_ANALOG_MODE = "set_analog_mode"
CLIENT_SET_DIGITAL_MODE = "set_digital_mode"


################################################################################
# ROS node
################################################################################


class ConductorManagerNode(rclpy.node.Node):
    """
    A ROS node that manages a LEGO train power conductor's conductor.
    """

    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Subsystems
        self._cpu_fan_manager: CPUFanManager = CPUFanManager(
            self, CPU_FAN_PWM_PIN, CPU_FAN_SPEED_PIN
        )
        self._mcu_memory_manager: McuMemoryManager = McuMemoryManager(self)
        self._sampling_manager: SamplingManager = SamplingManager(self)

        # Initialize hardware state
        self._supply_voltage: float = 0.0
        self._motor_voltage: float = 0.0
        self._motor_current: float = 0.0
        self._motor_ff1_state: bool = False
        self._motor_ff1_count: int = 0
        self._motor_ff2_state: bool = False
        self._motor_ff2_count: int = 0

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
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._conductor_state_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=ConductorStateMsg,
            topic=PUBLISH_CONDUCTOR_STATE,
            qos_profile=qos_profile,
        )

        # Subscribers
        self._analog_reading_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=AnalogReadingMsg,
                topic=SUBSCRIBE_ANALOG_READING,
                callback=self._on_analog_reading,
                qos_profile=qos_profile,
            )
        )
        self._digital_reading_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=DigitalReadingMsg,
                topic=SUBSCRIBE_DIGITAL_READING,
                callback=self._on_digital_reading,
                qos_profile=qos_profile,
            )
        )
        self._peripheral_input_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=PeripheralInputMsg,
                topic=SUBSCRIBE_PERIPHERAL_INPUT,
                callback=self._on_peripheral_input,
                qos_profile=qos_profile,
            )
        )
        self._peripherals_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=PeripheralScanMsg,
                topic=SUBSCRIBE_PERIPHERALS,
                callback=self._on_peripheral_scan,
                qos_profile=qos_profile,
            )
        )

        # Services
        self._power_control_service: rclpy.service.Service = self.create_service(
            srv_type=PowerControlSvc,
            srv_name=SERVICE_POWER_CONTROL,
            callback=self._handle_power_control,
        )

        # Service clients
        self._capture_input_client: rclpy.client.Client = self.create_client(
            srv_type=CaptureInputSvc, srv_name=CLIENT_CAPTURE_INPUT
        )
        self._digital_write_client: rclpy.client.Client = self.create_client(
            srv_type=DigitalWriteSvc, srv_name=CLIENT_DIGITAL_WRITE
        )
        self._pwm_write_client: rclpy.client.Client = self.create_client(
            srv_type=PWMWriteSvc, srv_name=CLIENT_PWM_WRITE
        )
        self._set_analog_mode_client: rclpy.client.Client = self.create_client(
            srv_type=SetAnalogModeSvc, srv_name=CLIENT_SET_ANALOG_MODE
        )
        self._set_digital_mode_client: rclpy.client.Client = self.create_client(
            srv_type=SetDigitalModeSvc, srv_name=CLIENT_SET_DIGITAL_MODE
        )

        # Timer parameters
        self._publish_timer: Optional[rclpy.node.Timer] = None

    def initialize(self) -> bool:
        self.get_logger().debug("Waiting for conductor services")
        self.get_logger().debug("  - Waiting for digital_write...")
        self._digital_write_client.wait_for_service()
        self.get_logger().debug("  - Waiting for pwm_write...")
        self._pwm_write_client.wait_for_service()
        self.get_logger().debug("  - Waiting for set_analog_mode...")
        self._set_analog_mode_client.wait_for_service()
        self.get_logger().debug("  - Waiting for set_digital_mode...")
        self._set_digital_mode_client.wait_for_service()

        self.get_logger().debug("Starting conductor configuration")

        # CPU fan
        if not self._cpu_fan_manager.initialize(CPU_FAN_SAMPLING_INTERVAL_MS):
            return False

        # Turn on fan
        self._cpu_fan_manager.write(CPU_FAN_PWM_PIN, 1.0)

        # Memory reporting
        if not self._mcu_memory_manager.initialize(REPORT_MCU_MEMORY_PERIOD_SECS):
            return False

        # Sampling interval
        if not self._sampling_manager.initialize(SAMPLING_INTERVAL_MS):
            return False

        # Voltage supply source (VSS)
        self.get_logger().debug(f"Enabling VSS on A{VSS_PIN}")
        if not self._set_analog_mode(VSS_PIN, AnalogMode.INPUT):
            return False

        #
        # Pololu md07a motor driver 18v15
        #

        # Motor PWM
        self.get_logger().debug(f"Enabling motor PWM on D{MOTOR_PWM_PIN}")
        if not self._set_digital_mode(MOTOR_PWM_PIN, DigitalMode.PWM):
            return False

        # Motor DIR
        self.get_logger().debug(f"Enabling motor DIR on D{MOTOR_DIR_PIN}")
        if not self._set_digital_mode(MOTOR_DIR_PIN, DigitalMode.OUTPUT):
            return False

        # Motor FF1
        self.get_logger().debug(f"Enabling motor FF1 on D{MOTOR_FF1_PIN}")
        if not self._set_digital_mode(MOTOR_FF1_PIN, DigitalMode.INPUT):
            return False

        # Motor FF2
        self.get_logger().debug(f"Enabling motor FF2 on D{MOTOR_FF2_PIN}")
        if not self._set_digital_mode(MOTOR_FF2_PIN, DigitalMode.INPUT):
            return False

        #
        # Sparkfun ACS712 current sensor
        #

        # Output voltage (VO)
        self.get_logger().debug(f"Enabling current sensor VO on A{MOTOR_CURRENT_PIN}")
        if not self._set_analog_mode(MOTOR_CURRENT_PIN, AnalogMode.INPUT):
            return False

        # Now that the manager is initialized, start the publishing timer
        self._publish_timer = self.create_timer(
            timer_period_sec=PUBLISH_STATE_PERIOD_SECS, callback=self._publish_state
        )

        self.get_logger().info("Conductor manager initialized successfully")

        return True

    def _set_analog_mode(self, analog_pin: int, analog_mode: AnalogMode) -> bool:
        # Create message
        vss_analog_svc = SetAnalogModeSvc.Request()
        vss_analog_svc.analog_pin = analog_pin
        vss_analog_svc.analog_mode = RosTranslator.analog_mode_to_ros(analog_mode)

        # Call service
        future: asyncio.Future = self._set_analog_mode_client.call_async(vss_analog_svc)

        # Wait for result
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

    def _set_digital_mode(self, digital_pin: int, digital_mode: DigitalMode) -> bool:
        # Create message
        motor_pwm_svc = SetDigitalModeSvc.Request()
        motor_pwm_svc.digital_pin = digital_pin
        motor_pwm_svc.digital_mode = RosTranslator.digital_mode_to_ros(digital_mode)

        # Call service
        future: asyncio.Future = self._set_digital_mode_client.call_async(motor_pwm_svc)

        # Wait for result
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

    def _on_analog_reading(self, analog_reading_msg: AnalogReadingMsg) -> None:
        analog_pin: int = analog_reading_msg.analog_pin
        reference_voltage: float = analog_reading_msg.reference_voltage
        analog_value: float = analog_reading_msg.analog_value

        # Translate analog value
        analog_voltage: float = analog_value * reference_voltage

        if analog_pin == VSS_PIN:
            # Apply voltage divider formula
            supply_voltage: float = analog_voltage * (VSS_R1 + VSS_R2) / VSS_R2

            # Calculate motor voltage
            motor_voltage = (
                supply_voltage * self._magnitude * (-1 if self._reverse else 1)
            )

            # Record state
            self._supply_voltage = supply_voltage
            self._motor_voltage = motor_voltage

        elif analog_pin == MOTOR_CURRENT_PIN:
            # TODO: Apply Vref and Gain to get current
            motor_current: float = analog_voltage

            # Record state
            self._motor_current = motor_current

    def _on_digital_reading(self, digital_reading_msg: DigitalReadingMsg) -> None:
        # Translate parameters
        digital_pin: int = digital_reading_msg.digital_pin
        digital_value: bool = digital_reading_msg.digital_value == 1

        if digital_pin == MOTOR_FF1_PIN:
            if digital_value != self._motor_ff1_state:
                # Increment count on high edge
                if digital_value:
                    self._motor_ff1_count += 1

                # Record state
                self._motor_ff1_state = digital_value
        elif digital_pin == MOTOR_FF2_PIN:
            if digital_value != self._motor_ff2_state:
                # Increment count on high edge
                if digital_value:
                    self._motor_ff2_count += 1

                # Record state
                self._motor_ff2_state = digital_value

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

            # Max thottle if hold speed is enabled
            if self._hold_speed:
                throttle = 1.0

            # Reduce throttle if B button is not pressed
            if not b_button:
                throttle *= 0.75  # Step 12V down to 9V

            magnitude: float = abs(throttle)
            reverse: bool = throttle < 0.0

            # Futures to wait on while the service is being called
            future_pwm: Optional[asyncio.Future] = None
            future_dir: Optional[asyncio.Future] = None

            # Update direction
            if self._reverse != reverse:
                self._reverse = reverse

                self.get_logger().info(
                    f"Direction: {'backward' if reverse else 'forward'}"
                )

                # Create message for motor direction
                digital_write_svc = DigitalWriteSvc.Request()
                digital_write_svc.digital_pin = MOTOR_DIR_PIN
                digital_write_svc.digital_value = 1 if reverse else 0

                # Call service
                future_dir = self._digital_write_client.call_async(digital_write_svc)

            # Update magnitude
            if self._magnitude != magnitude:
                self._magnitude = magnitude

                self.get_logger().info(f"Throttle: {throttle}")

                # Create message for motor PWM
                pwm_write_svc = PWMWriteSvc.Request()
                pwm_write_svc.digital_pin = MOTOR_PWM_PIN
                pwm_write_svc.duty_cycle = magnitude

                # Call service
                future_pwm = self._pwm_write_client.call_async(pwm_write_svc)

            # Update state
            self._motor_voltage = (
                self._supply_voltage * magnitude * (-1 if reverse else 1)
            )

            # Wait for results
            if future_pwm is not None:
                future_pwm.result()
            if future_dir is not None:
                future_dir.result()

    def _on_peripheral_scan(self, peripheral_scan_msg: PeripheralScanMsg) -> None:
        peripheral: PeripheralInfoMsg

        # Check for newly connected joysticks
        for peripheral in peripheral_scan_msg.peripherals:
            # Translate parameters
            peripheral_address: str = peripheral.address
            controller_profile: str = peripheral.controller_profile

            # Look for joystick types
            if peripheral.type != PeripheralConstantsMsg.TYPE_JOYSTICK:
                continue

            # Check if joystick is already open
            if peripheral_address in self._joysticks:
                continue

            # Update peripheral state
            self._joysticks[peripheral_address] = CONTROLLER_PROFILE

            # Open the joystick to capture input
            self._open_joystick(peripheral_address, controller_profile)

        # Check for disconnected joysticks
        joysticks_copy = self._joysticks.copy()
        for address, profile in joysticks_copy.items():
            found: bool = any(
                peripheral.address == address
                for peripheral in peripheral_scan_msg.peripherals
            )
            if not found:
                self.get_logger().debug(f"Closing joystick {address} of type {profile}")
                del self._joysticks[address]

    def _open_joystick(self, peripheral_address: str, controller_profile: str) -> None:
        self.get_logger().debug(
            f"Opening joystick {peripheral_address} of type {controller_profile}"
        )

        # Create message
        capture_input_svc = CaptureInputSvc.Request()
        capture_input_svc.capture = 1
        capture_input_svc.peripheral_address = peripheral_address
        capture_input_svc.controller_profile = CONTROLLER_PROFILE

        # Call service
        future: asyncio.Future = self._capture_input_client.call_async(
            capture_input_svc
        )

        # Wait for result
        future.result()

    def _handle_power_control(
        self, request: PowerControlSvc.Request, response: PowerControlSvc.Response
    ) -> PowerControlSvc.Response:
        power_mode: bool = request.power_mode == PowerModeMsg.ON

        self.get_logger().info(
            f"Received power {'on' if power_mode else 'off'} command"
        )

        # TODO
        # self._pwm_write_client()

        return response

    def _publish_state(self) -> None:
        header = HeaderMsg()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = NODE_NAME  # TODO

        msg: ConductorStateMsg = ConductorStateMsg()
        msg.header = header
        msg.supply_voltage = self._supply_voltage
        msg.motor_voltage = self._motor_voltage
        msg.motor_current = self._motor_current
        msg.motor_ff1_count = self._motor_ff1_count
        msg.motor_ff2_count = self._motor_ff2_count
        msg.cpu_fan_speed_rpm = self._cpu_fan_manager.cpu_fan_rpm
        msg.total_ram = self._mcu_memory_manager.total_ram
        msg.ram_utilization = self._mcu_memory_manager.ram_utilization

        self._conductor_state_pub.publish(msg)
