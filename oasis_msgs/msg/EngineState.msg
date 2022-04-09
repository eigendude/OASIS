################################################################################
#
#  Copyright (C) 2022 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# State of a LEGO train's main engine
#

# Message header and timestamp
std_msgs/Header header

# Total RAM size of the MCU, in bytes
uint32 total_ram

# Percent of RAM occupied by the system
float32 ram_utilization

# Supply voltage of the power source
float32 supply_voltage

# The last message logged from the microcontroller, or empty if a message
# hasn't been logged yet
string message
