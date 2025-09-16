################################################################################
#
#  Copyright (C) 2022-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Peripheral information
#

# The type of peripheral (See PeripheralConstants.msg)
uint8 type

# The name of the peripheral as reported by the driver
string name

# Logical address of the peripheral in the system, used to open/close
# peripherals
string address

# USB Vendor ID (integer, 0...65535), or 0 if unknown
uint16 vendor_id

# USB Product ID (integer, 0...65535), or 0 if unknown
uint16 product_id

# The Kodi controller profile that best represents this peripheral, of empty if
# unknown
string controller_profile

# Set to 0 if the peripheral is available to capture input, or 1 if already
# capturing input
uint8 capturing_input
