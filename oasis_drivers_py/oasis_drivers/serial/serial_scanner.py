################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from typing import List
from typing import Optional
from typing import Sequence

import serial.tools.list_ports
import serial.tools.list_ports_common

from oasis_drivers.serial.serial_types import SerialPort
from oasis_drivers.serial.serial_types import UsbDevice


class SerialScanner(object):
    """
    Utility class to scan for serial port devices using pyserial
    """

    @classmethod
    def do_scan(cls) -> List[SerialPort]:
        ports: Sequence[serial.tools.list_ports_common.ListPortInfo] = (
            serial.tools.list_ports.comports(include_links=False)
        )
        return [cls._get_serial_port(port) for port in ports]

    @classmethod
    def _get_serial_port(
        cls, port_info: serial.tools.list_ports_common.ListPortInfo
    ) -> SerialPort:
        port = SerialPort(
            device=port_info.device,
            name=port_info.name,
            description=port_info.description,
            hardware_id=port_info.hwid,
            usb_device=cls._get_usb_device(port_info),
        )
        return port

    @staticmethod
    def _get_usb_device(
        port_info: serial.tools.list_ports_common.ListPortInfo,
    ) -> Optional[UsbDevice]:
        if port_info.vid is not None and port_info.pid is not None:
            usb_device = UsbDevice(
                vendor_id=port_info.vid,
                product_id=port_info.pid,
                serial_number=(
                    port_info.serial_number if port_info.serial_number else ""
                ),
                location=port_info.location,
                manufacturer=port_info.manufacturer,
                product=port_info.product,
            )
            return usb_device
        return None
