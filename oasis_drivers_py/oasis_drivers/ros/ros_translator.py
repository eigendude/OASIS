################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from typing import List

from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Header as HeaderMsg

from oasis_drivers.serial.serial_types import SerialPort
from oasis_drivers.serial.serial_types import UsbDevice
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
        msg.location = usb_device.location
        msg.manufacturer = usb_device.manufacturer
        msg.product = usb_device.product

        return msg
