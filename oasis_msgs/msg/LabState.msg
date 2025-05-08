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
# State of a LEGO lab for driver development
#

# Message header and timestamp
std_msgs/Header header

# Total RAM size of the MCU, in bytes
uint32 total_ram

# Percent of RAM occupied by the system
float32 ram_utilization

# Output voltage of the current sensor (V)
float32 current_vout

# Current reading of the current sensor connected to the 5V line (Amp)
float32 shunt_current

# Output voltage of the IR reflectance sensor (V)
float32 ir_vout

# CO2 concentration (ppb)
float32 co2_ppb

# TVOC concentration (ppb)
float32 tvoc_ppb

# Acceleration
geometry_msgs/Vector3 linear_acceleration

# Rotation
geometry_msgs/Vector3 angular_velocity
