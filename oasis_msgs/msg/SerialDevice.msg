################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Information about a serial port
#

# Full device name/path, e.g. /dev/ttyUSB0
string device

# Short device name, e.g. ttyUSB0
string name

# Human readable description or empty
string description

# Technical description or empty
string hardware_id

# Optional USB-specific data, this is empty if it is not a USB device (or the
# platform does not support extended info)
UsbDevice[] usb_device
