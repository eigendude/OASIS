################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# USB-specific data
#

# USB Vendor ID (integer, 0...65535)
uint16 vendor_id

# USB Product ID (integer, 0...65535)
uint16 product_id

# USB serial number as a string
string serial_number

# USB device location string ("<bus>-<port>[-<port>]...")
string location

# USB manufacturer string, as reported by device
string manufacturer

# USB product string, as reported by device
string product
