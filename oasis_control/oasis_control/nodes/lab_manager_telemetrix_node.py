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
# Manager for a LEGO train's lab
#

import asyncio
from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription
from geometry_msgs.msg import Vector3 as Vector3Msg
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Header as HeaderMsg

from oasis_control.managers.ccs811_manager import CCS811Manager
from oasis_control.managers.mcu_memory_manager_telemetrix import McuMemoryManager
from oasis_control.managers.mpu6050_manager import MPU6050Manager
from oasis_control.managers.sampling_manager import SamplingManager
from oasis_drivers.ros.ros_translator import RosTranslator
from oasis_drivers.telemetrix.telemetrix_types import AnalogMode
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import LabState as LabStateMsg
from oasis_msgs.srv import DigitalWrite as DigitalWriteSvc
from oasis_msgs.srv import SetAnalogMode as SetAnalogModeSvc
from oasis_msgs.srv import SetDigitalMode as SetDigitalModeSvc


################################################################################
# Hardware configuration
################################################################################


# Sampling interval, in ms
SAMPLING_INTERVAL_MS: int = 100

# Memory reporting interval, in seconds
MEMORY_INTERVAL_SECS: float = 10.0  # RAM utilization doesn't currently change

# Pins
CURRENT_PIN: int = 0  # A0
IR_PIN: int = 1  # A1
RED_LED_1_PIN = 11  # D11
RED_LED_2_PIN = 2  # D2

# Shunt resistor value
SHUNT_RESISTOR_OHMS: float = 1.1

# Upper IR voltage threshold for reflectance
IR_THRESHOLD_VOLTS: float = 3.5

# I2C addresses
I2C_PORT: int = 0
I2C_CCS811_ADDRESS: int = 0x5B
I2C_MPU6050_ADDRESS: int = 0x68


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = "lab_manager_telemetrix"

PUBLISH_STATE_PERIOD_SECS = 0.1

# Publisher
PUBLISH_LAB_STATE = "lab_state"

# Subscribers
SUBSCRIBE_ANALOG_READING = "analog_reading"

# Service clients
CLIENT_DIGITAL_WRITE = "digital_write"
CLIENT_SET_ANALOG_MODE = "set_analog_mode"
CLIENT_SET_DIGITAL_MODE = "set_digital_mode"


################################################################################
# ROS node
################################################################################


class LabManagerNode(rclpy.node.Node):
    """
    A ROS node that manages a LEGO air quality lab.
    """

    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Subsystems
        """
        self._ccs811_manager: CCS811Manager = CCS811Manager(
            self, I2C_PORT, I2C_CCS811_ADDRESS
        )
        self._mpu6050_manager: MPU6050Manager = MPU6050Manager(
            self, I2C_PORT, I2C_MPU6050_ADDRESS
        )
        """
        self._mcu_memory_manager: McuMemoryManager = McuMemoryManager(self)
        self._sampling_manager: SamplingManager = SamplingManager(self)

        # Initialize hardware state
        self._current_vout: float = 0.0
        self._shunt_current: float = 0.0
        self._ir_vout: float = 0.0
        self._red_led_1_on = False
        self._red_led_2_on = False

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._lab_state_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=LabStateMsg,
            topic=PUBLISH_LAB_STATE,
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

        # Service clients
        self._digital_write_client: rclpy.client.Client = self.create_client(
            srv_type=DigitalWriteSvc, srv_name=CLIENT_DIGITAL_WRITE
        )
        self._set_analog_mode_client: rclpy.client.Client = self.create_client(
            srv_type=SetAnalogModeSvc, srv_name=CLIENT_SET_ANALOG_MODE
        )
        self._set_digital_mode_client: rclpy.client.Client = self.create_client(
            srv_type=SetDigitalModeSvc, srv_name=CLIENT_SET_DIGITAL_MODE
        )

        # Timer parameters
        self._publish_state_timer: Optional[rclpy.node.Timer] = None

    def initialize(self) -> bool:
        self.get_logger().debug("Waiting for Lab manager services")
        self.get_logger().debug("  - Waiting for digital_write...")
        self._digital_write_client.wait_for_service()
        self.get_logger().debug("  - Waiting for set_analog_mode...")
        self._set_analog_mode_client.wait_for_service()
        self.get_logger().debug("  - Waiting for set_digital_mode...")
        self._set_digital_mode_client.wait_for_service()

        self.get_logger().debug("Starting lab manager configuration")

        # Air quality
        # if not self._ccs811_manager.initialize():
        #     return False

        # IMU
        # if not self._mpu6050_manager.initialize():
        #     return False

        # Memory reporting
        if not self._mcu_memory_manager.initialize(MEMORY_INTERVAL_SECS):
            return False

        # Sampling interval
        if not self._sampling_manager.initialize(SAMPLING_INTERVAL_MS):
            return False

        # Vout (output of the shunt current sensor)
        self.get_logger().debug(f"Enabling Vout on A{CURRENT_PIN}")
        if not self._set_analog_mode(CURRENT_PIN, AnalogMode.INPUT):
            return False

        # Vout (output of the IR sensor)
        self.get_logger().debug(f"Enabling Vout on A{IR_PIN}")
        if not self._set_analog_mode(IR_PIN, AnalogMode.INPUT):
            return False

        # Red LED 1
        self.get_logger().debug(f"Enabling red LED 1 on D{RED_LED_1_PIN}")
        if not self._set_digital_mode(RED_LED_1_PIN, DigitalMode.OUTPUT):
            return False

        # Red LED 2
        self.get_logger().debug(f"Enabling red LED 2 on D{RED_LED_2_PIN}")
        if not self._set_digital_mode(RED_LED_2_PIN, DigitalMode.OUTPUT):
            return False

        # Now that the manager is initialized, start the publishing timer
        self._publish_state_timer = self.create_timer(
            timer_period_sec=PUBLISH_STATE_PERIOD_SECS, callback=self._publish_state
        )

        self.get_logger().info("Lab manager initialized successfully")

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
        # Translate parameters
        analog_pin: int = analog_reading_msg.analog_pin
        reference_voltage: float = analog_reading_msg.reference_voltage
        analog_value: float = analog_reading_msg.analog_value

        # Translate analog value
        if not 0.0 <= analog_value <= 1.0:
            self.get_logger().warning(
                f"Analog reading {analog_value:.2f} for pin {analog_pin} out of "
                "range, clamping to [0.0, 1.0]"
            )
        normalized_value = max(0.0, min(analog_value, 1.0))
        analog_voltage: float = normalized_value * reference_voltage

        if analog_pin == CURRENT_PIN:
            # Record state
            self._current_vout = analog_voltage

            # Follow the equation given by the INA169 datasheet to determine the
            # current flowing through the shunt resistor RS. Assume RL = 10k.
            #
            # Is = (Vout * 1k) / (RS * RL)
            self._shunt_current = self._current_vout / (SHUNT_RESISTOR_OHMS * 10)
        elif analog_pin == IR_PIN:
            # Record state
            self._ir_vout = analog_voltage

            # Create message for motor direction
            digital_write_svc = DigitalWriteSvc.Request()

            # Check IR threshold
            if self._ir_vout > IR_THRESHOLD_VOLTS:
                # Turn both LEDs off
                if self._red_led_1_on:
                    self._red_led_1_on = False

                    digital_write_svc.digital_pin = RED_LED_1_PIN
                    digital_write_svc.digital_value = 0

                    # Call service
                    future_write = self._digital_write_client.call_async(
                        digital_write_svc
                    )

                    # Wait for result
                    if future_write is not None:
                        future_write.result()
                if self._red_led_2_on:
                    self._red_led_2_on = False

                    digital_write_svc.digital_pin = RED_LED_2_PIN
                    digital_write_svc.digital_value = 0

                    # Call service
                    future_write = self._digital_write_client.call_async(
                        digital_write_svc
                    )

                    # Wait for result
                    if future_write is not None:
                        future_write.result()
            else:
                # Turn both LEDs on
                if not self._red_led_1_on:
                    self._red_led_1_on = True

                    digital_write_svc.digital_pin = RED_LED_1_PIN
                    digital_write_svc.digital_value = 1

                    # Call service
                    future_write = self._digital_write_client.call_async(
                        digital_write_svc
                    )

                    # Wait for result
                    if future_write is not None:
                        future_write.result()
                if not self._red_led_2_on:
                    self._red_led_2_on = True

                    digital_write_svc.digital_pin = RED_LED_2_PIN
                    digital_write_svc.digital_value = 1

                    # Call service
                    future_write = self._digital_write_client.call_async(
                        digital_write_svc
                    )

                    # Wait for result
                    if future_write is not None:
                        future_write.result()

    def _publish_state(self) -> None:
        header = HeaderMsg()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = NODE_NAME  # TODO

        msg: LabStateMsg = LabStateMsg()
        msg.header = header
        msg.total_ram = self._mcu_memory_manager.total_ram
        msg.ram_utilization = self._mcu_memory_manager.ram_utilization
        msg.current_vout = self._current_vout
        msg.shunt_current = self._shunt_current
        msg.ir_vout = self._ir_vout
        # msg.co2_ppb = self._ccs811_manager.co2_ppb
        # msg.tvoc_ppb = self._ccs811_manager.tvoc_ppb
        msg.linear_acceleration = Vector3Msg()
        msg.linear_acceleration.x = self._mpu6050_manager.ax
        msg.linear_acceleration.y = self._mpu6050_manager.ay
        msg.linear_acceleration.z = self._mpu6050_manager.az
        msg.angular_velocity = Vector3Msg()
        msg.angular_velocity.x = self._mpu6050_manager.gx
        msg.angular_velocity.y = self._mpu6050_manager.gy
        msg.angular_velocity.z = self._mpu6050_manager.gz

        self._lab_state_pub.publish(msg)
