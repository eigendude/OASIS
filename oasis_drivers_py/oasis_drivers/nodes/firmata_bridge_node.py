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
from oasis_msgs.msg import DigitalReading as DigitalReadingMsg
from oasis_msgs.srv import AnalogRead as AnalogReadSvc
from oasis_msgs.srv import DigitalRead as DigitalReadSvc
from oasis_msgs.srv import DigitalWrite as DigitalWriteSvc
from oasis_msgs.srv import PWMWrite as PWMWriteSvc
from oasis_msgs.srv import ServoWrite as ServoWriteSvc
from oasis_msgs.srv import SetAnalogMode as SetAnalogModeSvc
from oasis_msgs.srv import SetDigitalMode as SetDigitalModeSvc


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "firmata_bridge"

# ROS topics
ANALOG_READING_TOPIC = "analog_reading"
DIGITAL_READING_TOPIC = "digital_reading"
STRING_MESSAGE_TOPIC = "string_message"

# ROS services
ANALOG_READ_SERVICE = "analog_read"
DIGITAL_READ_SERVICE = "digital_read"
DIGITAL_WRITE_SERVICE = "digital_write"
PWM_WRITE_SERVICE = "pwm_write"
SERVO_WRITE_SERVICE = "servo_write"
SET_ANALOG_MODE_SERVICE = "set_analog_mode"
SET_DIGITAL_MODE_SERVICE = "set_digital_mode"


################################################################################
# ROS node
################################################################################


class FirmataBridgeNode(rclpy.node.Node, FirmataCallback):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Initialize members
        self._bridge = FirmataBridge(self)

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
        self._digital_reading_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=DigitalReadingMsg,
            topic=DIGITAL_READING_TOPIC,
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

        # Perform service
        self._bridge.pwm_write(digital_pin, duty_cycle)

        return response

    def _handle_servo_write(
        self, request: ServoWriteSvc.Request, response: ServoWriteSvc.Response
    ) -> ServoWriteSvc.Response:
        """Handle ROS 2 servo pin writes"""
        # Translate parameters
        digital_pin: int = request.digital_pin
        position: float = request.position

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
