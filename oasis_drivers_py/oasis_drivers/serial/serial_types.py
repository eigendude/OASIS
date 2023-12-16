#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import collections


################################################################################
# Serial port types
################################################################################

SerialPort = collections.namedtuple(
    "SerialPort",
    [
        "device",
        "name",
        "description",
        "hardware_id",
        "usb_device",
    ],
)


UsbDevice = collections.namedtuple(
    "UsbDevice",
    [
        "vendor_id",
        "product_id",
        "serial_number",
        "location",
        "manufacturer",
        "product",
    ],
)
