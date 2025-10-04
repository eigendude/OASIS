################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
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
from typing import Union

from oasis_drivers.firmata.firmata_types import AnalogMode as FirmataAnalogMode
from oasis_drivers.firmata.firmata_types import DigitalMode as FirmataDigitalMode
from oasis_drivers.telemetrix.telemetrix_types import AnalogMode as TelemetrixAnalogMode
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode as TelemetrixDigitalMode
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
    def analog_mode_to_ros(
        analog_mode: Union[FirmataAnalogMode, TelemetrixAnalogMode]
    ) -> int:
        return {
            TelemetrixAnalogMode.DISABLED.value: AVRConstantsMsg.ANALOG_DISABLED,
            TelemetrixAnalogMode.INPUT.value: AVRConstantsMsg.ANALOG_INPUT,
        }[analog_mode.value]

    @staticmethod
    def digital_mode_to_ros(
        digital_mode: Union[FirmataDigitalMode, TelemetrixDigitalMode]
    ) -> int:
        return {
            TelemetrixDigitalMode.DISABLED.value: AVRConstantsMsg.DIGITAL_DISABLED,
            TelemetrixDigitalMode.INPUT.value: AVRConstantsMsg.DIGITAL_INPUT,
            TelemetrixDigitalMode.INPUT_PULLUP.value: AVRConstantsMsg.DIGITAL_INPUT_PULLUP,
            TelemetrixDigitalMode.OUTPUT.value: AVRConstantsMsg.DIGITAL_OUTPUT,
            TelemetrixDigitalMode.PWM.value: AVRConstantsMsg.DIGITAL_PWM,
            TelemetrixDigitalMode.SERVO.value: AVRConstantsMsg.DIGITAL_SERVO,
            TelemetrixDigitalMode.CPU_FAN_PWM.value: AVRConstantsMsg.DIGITAL_CPU_FAN_PWM,
            TelemetrixDigitalMode.CPU_FAN_TACHOMETER.value: AVRConstantsMsg.DIGITAL_CPU_FAN_TACHOMETER,
        }[digital_mode.value]

    @staticmethod
    def analog_mode_to_telemetrix(
        ros2_analog_mode: int
    ) -> TelemetrixAnalogMode:
        """Translate an analog pin mode from ROS 2 API to Telemetrix API"""
        return {
            AVRConstantsMsg.ANALOG_DISABLED: TelemetrixAnalogMode.DISABLED,
            AVRConstantsMsg.ANALOG_INPUT: TelemetrixAnalogMode.INPUT,
        }[ros2_analog_mode]

    @staticmethod
    def digital_mode_to_telemetrix(
        ros2_digital_mode: int
    ) -> TelemetrixDigitalMode:
        """Translate a digital pin mode from ROS 2 API to Telemetrix API"""
        return {
            AVRConstantsMsg.DIGITAL_DISABLED: TelemetrixDigitalMode.DISABLED,
            AVRConstantsMsg.DIGITAL_INPUT: TelemetrixDigitalMode.INPUT,
            AVRConstantsMsg.DIGITAL_INPUT_PULLUP: TelemetrixDigitalMode.INPUT_PULLUP,
            AVRConstantsMsg.DIGITAL_OUTPUT: TelemetrixDigitalMode.OUTPUT,
            AVRConstantsMsg.DIGITAL_PWM: TelemetrixDigitalMode.PWM,
            AVRConstantsMsg.DIGITAL_SERVO: TelemetrixDigitalMode.SERVO,
            AVRConstantsMsg.DIGITAL_CPU_FAN_PWM: TelemetrixDigitalMode.CPU_FAN_PWM,
            AVRConstantsMsg.DIGITAL_CPU_FAN_TACHOMETER: TelemetrixDigitalMode.CPU_FAN_TACHOMETER,
        }[ros2_digital_mode]

    @staticmethod
    def convert_timestamp(timestamp: datetime) -> TimeMsg:
        """Convert datetime to ROS 2 time message"""
        return rclpy.time.Time(
            seconds=int(timestamp.timestamp()),
            nanoseconds=timestamp.microsecond * 1000,
        ).to_msg()
