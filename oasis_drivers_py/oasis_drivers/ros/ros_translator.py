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
from typing import List

import rclpy.time
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Header as HeaderMsg

from oasis_drivers.serial.serial_types import SerialPort
from oasis_drivers.serial.serial_types import UsbDevice
from oasis_drivers.telemetrix.telemetrix_types import AnalogMode
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode
from oasis_msgs.msg import AVRConstants as AVRConstantsMsg
from oasis_msgs.msg import SerialDevice as SerialDeviceMsg
from oasis_msgs.msg import SerialDeviceScan as SerialDeviceScanMsg
from oasis_msgs.msg import UsbDevice as UsbDeviceMsg


class RosTranslator(object):
    """
    Container for translating ROS values and types.
    """

    @classmethod
    def get_serial_ports_msg(
        cls, ports: List[SerialPort], timestamp_msg: TimeMsg
    ) -> SerialDeviceScanMsg:
        msg = SerialDeviceScanMsg()

        msg.header = HeaderMsg()
        msg.header.stamp = timestamp_msg
        msg.header.frame_id = ""  # TODO
        msg.serial_devices = [cls._get_serial_port_msg(port) for port in ports]

        return msg

    @classmethod
    def _get_serial_port_msg(cls, port: SerialPort) -> SerialDeviceMsg:
        msg = SerialDeviceMsg()

        msg.device = port.device
        msg.name = port.name
        msg.description = port.description
        msg.hardware_id = port.hardware_id
        if port.usb_device:
            msg.usb_device = [cls._get_usb_device_msg(port.usb_device)]
        else:
            msg.usb_device = []

        return msg

    @staticmethod
    def _get_usb_device_msg(usb_device: UsbDevice) -> UsbDeviceMsg:
        msg = UsbDeviceMsg()

        msg.vendor_id = usb_device.vendor_id
        msg.product_id = usb_device.product_id
        msg.serial_number = usb_device.serial_number if usb_device.serial_number else ""
        msg.location = usb_device.location if usb_device.location else ""
        msg.manufacturer = usb_device.manufacturer if usb_device.manufacturer else ""
        msg.product = usb_device.product if usb_device.product else ""

        return msg

    @staticmethod
    def analog_mode_to_ros(analog_mode: AnalogMode) -> int:
        return {
            AnalogMode.DISABLED: AVRConstantsMsg.ANALOG_DISABLED,
            AnalogMode.INPUT: AVRConstantsMsg.ANALOG_INPUT,
        }[analog_mode]

    @staticmethod
    def digital_mode_to_ros(digital_mode: DigitalMode) -> int:
        return {
            DigitalMode.DISABLED: AVRConstantsMsg.DIGITAL_DISABLED,
            DigitalMode.INPUT: AVRConstantsMsg.DIGITAL_INPUT,
            DigitalMode.INPUT_PULLUP: AVRConstantsMsg.DIGITAL_INPUT_PULLUP,
            DigitalMode.OUTPUT: AVRConstantsMsg.DIGITAL_OUTPUT,
            DigitalMode.PWM: AVRConstantsMsg.DIGITAL_PWM,
            DigitalMode.SERVO: AVRConstantsMsg.DIGITAL_SERVO,
            DigitalMode.CPU_FAN_PWM: AVRConstantsMsg.DIGITAL_CPU_FAN_PWM,
            DigitalMode.CPU_FAN_TACHOMETER: AVRConstantsMsg.DIGITAL_CPU_FAN_TACHOMETER,
        }[digital_mode]

    @staticmethod
    def analog_mode_to_telemetrix(ros2_analog_mode: int) -> AnalogMode:
        """Translate an analog pin mode from ROS 2 API to Telemetrix API"""
        return {
            AVRConstantsMsg.ANALOG_DISABLED: AnalogMode.DISABLED,
            AVRConstantsMsg.ANALOG_INPUT: AnalogMode.INPUT,
        }[ros2_analog_mode]

    @staticmethod
    def digital_mode_to_telemetrix(ros2_digital_mode: int) -> DigitalMode:
        """Translate a digital pin mode from ROS 2 API to Telemetrix API"""
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

    @staticmethod
    def convert_timestamp(timestamp: datetime) -> TimeMsg:
        """Convert datetime to ROS 2 time message"""
        return rclpy.time.Time(
            seconds=int(timestamp.timestamp()),
            nanoseconds=timestamp.microsecond * 1000,
        ).to_msg()
