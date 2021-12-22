################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from dataclasses import dataclass
from typing import Dict
from typing import Optional

import pyudev


@dataclass
class UdevDevice:
    """
    A single device with attached attributes and properties.
    """

    id: str
    path: str
    subsystem: str
    name: str
    product: str
    manufacturer: str
    vendor_id: str
    product_id: str
    serial: Optional[str]
    properties: Dict[str, str]

    @staticmethod
    def from_udev_device(device: pyudev.Device) -> "UdevDevice":
        # For the serial number, use ID_SERIAL_SHORT instead of ID_SERIAL.
        #
        # Udev generates the long-form serial number by concatenating vendor,
        # model and (if available) short serial with "_". This guarantees a
        # non-empty serial number in case the short-form serial is missing.
        # However we don't want this extra stuff, so use the short-form serial
        # number and add handling for the case of a missing serial number.
        serial_number = device.properties.get("ID_SERIAL_SHORT")

        return UdevDevice(
            id=device.device_path,
            path=device.device_node,
            subsystem=device.properties.get("SUBSYSTEM"),
            name=device.properties.get("NAME"),
            product=device.properties.get("ID_MODEL"),
            manufacturer=device.properties.get("ID_VENDOR"),
            vendor_id=device.properties.get("ID_VENDOR_ID"),
            product_id=device.properties.get("ID_MODEL_ID"),
            serial=serial_number,
            properties=device.properties,
        )

    def is_valid(self) -> bool:
        return bool(self.path)

    def to_string(self) -> str:
        return (
            "\n"
            + f"  path={self.path}\n"
            + f"  subsystem={self.subsystem}\n"
            + f"  name={self.name}\n"
            + f"  product={self.product}\n"
            + f"  manufacturer={self.manufacturer}\n"
            + f"  usb_id={self.vendor_id}:{self.product_id}\n"
            + f"  serial={self.serial}\n"
            + f"  num_properties={len(self.properties)}"
        )
