################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Event occurring while powering a device on or off
#

# Timestamp and frame_id
std_msgs/Header header

# The machine hosting the device being powered
string system

# Path to the device being powered
string device

# The new power state of type PowerMode
string power_mode
