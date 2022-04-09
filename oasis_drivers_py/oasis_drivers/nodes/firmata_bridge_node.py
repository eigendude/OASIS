################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from datetime import datetime
from typing import Tuple

import rclpy.node
import rclpy.service
import rclpy.time
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Header as HeaderMsg
from std_msgs.msg import String as StringMsg

from oasis_drivers.firmata.firmata_bridge import FirmataBridge
from oasis_drivers.firmata.firmata_callback import FirmataCallback
from oasis_drivers.firmata.firmata_types import AnalogMode
from oasis_drivers.firmata.firmata_types import DigitalMode
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import AVRConstants as AVRConstantsMsg
from oasis_msgs.msg import CPUFanSpeed as CPUFanSpeedMsg
from oasis_msgs.msg import DigitalReading as DigitalReadingMsg
from oasis_msgs.msg import MCUMemory as MCUMemoryMsg
from oasis_msgs.srv import AnalogRead as AnalogReadSvc
from oasis_msgs.srv import DigitalRead as DigitalReadSvc
from oasis_msgs.srv import DigitalWrite as DigitalWriteSvc
from oasis_msgs.srv import PWMWrite as PWMWriteSvc
from oasis_msgs.srv import ReportMCUMemory as ReportMCUMemorySvc
from oasis_msgs.srv import ServoWrite as ServoWriteSvc
from oasis_msgs.srv import SetAnalogMode as SetAnalogModeSvc
from oasis_msgs.srv import SetDigitalMode as SetDigitalModeSvc


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "firmata_bridge"

# ROS parameters
PARAM_COM_PORT = "com_port"

# ROS topics
ANALOG_READING_TOPIC = "analog_reading"
CPU_FAN_SPEED_TOPIC = "cpu_fan_speed"
DIGITAL_READING_TOPIC = "digital_reading"
MCU_MEMORY_TOPIC = "mcu_memory"
STRING_MESSAGE_TOPIC = "string_message"

# ROS services
ANALOG_READ_SERVICE = "analog_read"
DIGITAL_READ_SERVICE = "digital_read"
DIGITAL_WRITE_SERVICE = "digital_write"
PWM_WRITE_SERVICE = "pwm_write"
REPORT_MCU_MEMORY_SERVICE = "report_mcu_memory"
SERVO_WRITE_SERVICE = "servo_write"
SET_ANALOG_MODE_SERVICE = "set_analog_mode"
SET_DIGITAL_MODE_SERVICE = "set_digital_mode"


################################################################################
# Firmata parameters
################################################################################


DEFAULT_COM_PORT = "/dev/ttyACM0"


################################################################################
# ROS node
################################################################################


class FirmataBridgeNode(rclpy.node.Node, FirmataCallback):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        # Initialize rclpy.node.NODE
        super().__init__(NODE_NAME)
        self.declare_parameter(PARAM_COM_PORT, DEFAULT_COM_PORT)

        # Initialize members
        com_port: str = str(self.get_parameter(PARAM_COM_PORT).value)
        self._bridge = FirmataBridge(self, com_port)

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._analog_reading_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=AnalogReadingMsg,
            topic=ANALOG_READING_TOPIC,
            qos_profile=qos_profile,
        )
        self._cpu_fan_speed_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=CPUFanSpeedMsg,
            topic=CPU_FAN_SPEED_TOPIC,
            qos_profile=qos_profile,
        )
        self._digital_reading_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=DigitalReadingMsg,
            topic=DIGITAL_READING_TOPIC,
            qos_profile=qos_profile,
        )
        self._mcu_memory_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=MCUMemoryMsg,
            topic=MCU_MEMORY_TOPIC,
            qos_profile=qos_profile,
        )
        # TODO: the String message was deprecated in Foxy. We should switch to
        # an application-specific message.
        self._string_message_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=StringMsg,
            topic=STRING_MESSAGE_TOPIC,
            qos_profile=qos_profile,
        )

        # Once the publishers are set up, start the bridge
        self._bridge.initialize()

        # Once the bridge is started, advertise the services
        self._analog_read_service: rclpy.service.Service = self.create_service(
            srv_type=AnalogReadSvc,
            srv_name=ANALOG_READ_SERVICE,
            callback=self._handle_analog_read,
        )
        self._digital_read_service: rclpy.service.Service = self.create_service(
            srv_type=DigitalReadSvc,
            srv_name=DIGITAL_READ_SERVICE,
            callback=self._handle_digital_read,
        )
        self._digital_write_service: rclpy.service.Service = self.create_service(
            srv_type=DigitalWriteSvc,
            srv_name=DIGITAL_WRITE_SERVICE,
            callback=self._handle_digital_write,
        )
        self._pwm_write_service: rclpy.service.Service = self.create_service(
            srv_type=PWMWriteSvc,
            srv_name=PWM_WRITE_SERVICE,
            callback=self._handle_pwm_write,
        )
        self._report_mcu_memory_service: rclpy.service.Service = self.create_service(
            srv_type=ReportMCUMemorySvc,
            srv_name=REPORT_MCU_MEMORY_SERVICE,
            callback=self._handle_report_mcu_memory,
        )
        self._servo_write_service: rclpy.service.Service = self.create_service(
            srv_type=ServoWriteSvc,
            srv_name=SERVO_WRITE_SERVICE,
            callback=self._handle_servo_write,
        )
        self._set_analog_mode_service: rclpy.service.Service = self.create_service(
            srv_type=SetAnalogModeSvc,
            srv_name=SET_ANALOG_MODE_SERVICE,
            callback=self._handle_set_analog_mode,
        )
        self._set_digital_mode_service: rclpy.service.Service = self.create_service(
            srv_type=SetDigitalModeSvc,
            srv_name=SET_DIGITAL_MODE_SERVICE,
            callback=self._handle_set_digital_mode,
        )

        self.get_logger().info("Firmata bridge initialized")

    def stop(self) -> None:
        """Stop the bridge and cleanup ROS resources"""
        self._bridge.deinitialize()

        self.get_logger().info("Firmata bridge deinitialized")

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        self.destroy_node()

    def on_analog_reading(
        self,
        timestamp: datetime,
        analog_pin: int,
        analog_value: float,
        reference_voltage: float,
    ) -> None:
        """Implement FirmataCallback"""
        msg: AnalogReadingMsg = AnalogReadingMsg()

        # Timestamp in ROS header
        header = HeaderMsg()
        header.stamp = self._convert_timestamp(timestamp)
        header.frame_id = ""  # TODO

        msg.header = header
        msg.analog_pin = analog_pin
        msg.reference_voltage = reference_voltage
        msg.analog_value = analog_value

        self._analog_reading_pub.publish(msg)

    def on_cpu_fan_rpm(self, timestamp: datetime, digital_pin: int, rpm: int) -> None:
        """Implement FirmataCallback"""
        msg: CPUFanSpeedMsg = CPUFanSpeedMsg()

        # Timestamp in ROS header
        header = HeaderMsg()
        header.stamp = self._convert_timestamp(timestamp)
        header.frame_id = ""  # TODO

        msg.header = header
        msg.digital_pin = digital_pin
        msg.fan_speed_rpm = rpm

        self._cpu_fan_speed_pub.publish(msg)

    def on_digital_reading(
        self, timestamp: datetime, digital_pin: int, digital_value: bool
    ) -> None:
        """Implement FirmataCallback"""
        msg: DigitalReadingMsg = DigitalReadingMsg()

        # Timestamp in ROS header
        header = HeaderMsg()
        header.stamp = self._convert_timestamp(timestamp)
        header.frame_id = ""  # TODO

        msg.header = header
        msg.digital_pin = digital_pin
        msg.digital_value = (
            AVRConstantsMsg.HIGH if digital_value else AVRConstantsMsg.LOW
        )

        self._digital_reading_pub.publish(msg)

    def on_memory_data(
        self,
        total_ram: int,
        static_data_size: int,
        heap_size: int,
        stack_size: int,
        free_ram: int,
        free_heap: int,
    ) -> None:
        """Implement FirmataCallback"""
        msg: MCUMemoryMsg = MCUMemoryMsg()

        # Timestamp in ROS header
        header = HeaderMsg()
        header.stamp = self._get_timestamp()
        header.frame_id = ""  # TODO

        msg.header = header
        msg.total_ram = total_ram
        msg.static_data_size = static_data_size
        msg.heap_size = heap_size
        msg.stack_size = stack_size
        msg.free_ram = free_ram
        msg.free_heap = free_heap

        self._mcu_memory_pub.publish(msg)

    def on_string_data(self, data: str) -> None:
        """Implement FirmataCallback"""
        msg: StringMsg = StringMsg()

        msg.data = data

        self._string_message_pub.publish(msg)

        # Debug logging
        self.get_logger().info(data)

    def _handle_analog_read(
        self, request: AnalogReadSvc.Request, response: AnalogReadSvc.Response
    ) -> AnalogReadSvc.Response:
        """Handle ROS 2 analog pin reads"""
        # Translate parameters
        analog_pin: int = request.analog_pin

        # Debug logging
        self.get_logger().info(f"Reading value for analog pin {analog_pin}")

        # Perform service
        result: Tuple[float, float, datetime] = self._bridge.analog_read(analog_pin)

        # Translate result
        response.stamp = self._convert_timestamp(result[2])
        response.reference_voltage = float(result[1])
        response.analog_value = float(result[0])

        return response

    def _handle_digital_read(
        self, request: DigitalReadSvc.Request, response: DigitalReadSvc.Response
    ) -> DigitalReadSvc.Response:
        """Handle ROS 2 digital pin reads"""
        # Translate parameters
        digital_pin: int = request.digital_pin

        # Debug logging
        self.get_logger().info(f"Reading value for digital pin {digital_pin}")

        # Perform service
        result: Tuple[bool, datetime] = self._bridge.digital_read(digital_pin)

        # Translate result
        response.stamp = self._convert_timestamp(result[1])
        response.value = AVRConstantsMsg.HIGH if result[0] else AVRConstantsMsg.LOW

        return response

    def _handle_digital_write(
        self, request: DigitalWriteSvc.Request, response: DigitalWriteSvc.Response
    ) -> DigitalWriteSvc.Response:
        """Handle ROS 2 digital pin writes"""
        # Translate parameters
        digital_pin: int = request.digital_pin
        digital_value: bool = request.digital_value == AVRConstantsMsg.HIGH

        # Debug logging
        self.get_logger().info(
            f"Setting digital pin {digital_pin} value to {'HIGH' if digital_value else 'LOW'}"
        )

        # Perform service
        self._bridge.digital_write(digital_pin, digital_value)

        return response

    def _handle_pwm_write(
        self, request: PWMWriteSvc.Request, response: PWMWriteSvc.Response
    ) -> PWMWriteSvc.Response:
        """Handle ROS 2 PWM pin writes"""
        # Translate parameters
        digital_pin: int = request.digital_pin
        duty_cycle: float = request.duty_cycle

        # Debug logging
        self.get_logger().info(
            f"Setting PWM on pin {digital_pin} to duty cycle {duty_cycle}"
        )

        # Perform service
        self._bridge.pwm_write(digital_pin, duty_cycle)

        return response

    def _handle_report_mcu_memory(
        self, request: ReportMCUMemorySvc.Request, response: ReportMCUMemorySvc.Response
    ) -> ReportMCUMemorySvc.Response:
        """Handle request to enable/disable MCU memory reporting"""
        # Translate parameters
        reporting_period_ms: int = request.reporting_period_ms

        # Debug logging
        if reporting_period_ms != 0:
            self.get_logger().info(f"Reporting MCU memory at {reporting_period_ms}ms")
        else:
            self.get_logger().info("Disabling MCU memory reporting")

        # Perform service
        self._bridge.report_mcu_memory(reporting_period_ms)

        return response

    def _handle_servo_write(
        self, request: ServoWriteSvc.Request, response: ServoWriteSvc.Response
    ) -> ServoWriteSvc.Response:
        """Handle ROS 2 servo pin writes"""
        # Translate parameters
        digital_pin: int = request.digital_pin
        position: float = request.position

        # Debug logging
        self.get_logger().info(
            f"Setting servo on pin {digital_pin} to position {position}"
        )

        # Perform service
        self._bridge.servo_write(digital_pin, position)

        return response

    def _handle_set_analog_mode(
        self, request: SetAnalogModeSvc.Request, response: SetAnalogModeSvc.Response
    ) -> SetAnalogModeSvc.Response:
        """Handle ROS 2 analog pin mode changes"""
        # Translate parameters
        analog_pin: int = request.analog_pin
        analog_mode: AnalogMode = self._translate_analog_mode(request.analog_mode)

        # Debug logging
        self.get_logger().info(f"Setting analog pin {analog_pin} to mode {analog_mode}")

        # Perform service
        self._bridge.set_analog_mode(analog_pin, analog_mode)

        return response

    def _handle_set_digital_mode(
        self, request: SetDigitalModeSvc.Request, response: SetDigitalModeSvc.Response
    ) -> SetDigitalModeSvc.Response:
        """Handle ROS 2 digital pin mode changes"""
        # Translate parameters
        digital_pin: int = request.digital_pin
        digital_mode: DigitalMode = self._translate_digital_mode(request.digital_mode)

        # Debug logging
        self.get_logger().info(
            f"Setting digital pin {digital_pin} to mode {digital_mode}"
        )

        # Perform service
        self._bridge.set_digital_mode(digital_pin, digital_mode)

        return response

    def _translate_analog_mode(self, ros2_analog_mode: int) -> AnalogMode:
        """Translate an analog pin mode from ROS 2 API to Firmata API"""
        try:
            return {
                AVRConstantsMsg.ANALOG_DISABLED: AnalogMode.DISABLED,
                AVRConstantsMsg.ANALOG_INPUT: AnalogMode.INPUT,
            }[ros2_analog_mode]
        except KeyError:
            self.get_logger().error(
                f"Invalid analog mode ({ros2_analog_mode}), disabling pin"
            )
            return AnalogMode.DISABLED

    def _translate_digital_mode(self, ros2_digital_mode: int) -> DigitalMode:
        """Translate a digital pin mode from ROS 2 API to Firmata API"""
        try:
            return {
                AVRConstantsMsg.DIGITAL_DISABLED: DigitalMode.DISABLED,
                AVRConstantsMsg.DIGITAL_INPUT: DigitalMode.INPUT,
                AVRConstantsMsg.DIGITAL_INPUT_PULLUP: DigitalMode.INPUT_PULLUP,
                AVRConstantsMsg.DIGITAL_OUTPUT: DigitalMode.OUTPUT,
                AVRConstantsMsg.DIGITAL_PWM: DigitalMode.PWM,
                AVRConstantsMsg.DIGITAL_SERVO: DigitalMode.SERVO,
                AVRConstantsMsg.DIGITAL_CPU_FAN_PWM: DigitalMode.CPU_FAN_PWM,
                AVRConstantsMsg.DIGITAL_CPU_FAN_TACHOMETER: DigitalMode.CPU_FAN_TACHOMETER,
            }[ros2_digital_mode]
        except KeyError:
            self.get_logger().error(
                f"Invalid digital mode ({ros2_digital_mode}), disabling pin"
            )
            return DigitalMode.DISABLED

    @staticmethod
    def _convert_timestamp(timestamp: datetime) -> TimeMsg:
        """Convert datetime to ROS 2 time message"""
        return rclpy.time.Time(
            seconds=int(timestamp.timestamp()),
            nanoseconds=timestamp.microsecond * 1000,
        ).to_msg()

    def _get_timestamp(self) -> TimeMsg:
        return self.get_clock().now().to_msg()
