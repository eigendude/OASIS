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
# State of the Leonardo attached to the NAS server.
#

# Message header and timestamp
std_msgs/Header header

# Reference voltage
float32 reference_voltage

# Poteniometer value
float32 potentiometer

# Total RAM size of the MCU, in bytes
uint32 total_ram

# Percent of RAM occupied by the system
float32 ram_utilization
