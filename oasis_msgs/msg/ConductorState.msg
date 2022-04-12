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
# State of the train power station
#

# Message header and timestamp
std_msgs/Header header

# 12V supply voltage, read via voltage divider
float32 supply_voltage

# Motor voltage, negative for the backwards direction
float32 motor_voltage

# Motor current, in Amps
float32 motor_current

# Number of times the motor driver FF1 pin has been triggered
uint32 motor_ff1_count

# Number of times the motor driver FF2 pin has been triggered
uint32 motor_ff2_count

# CPU fan speed (RPM)
uint32 cpu_fan_speed_rpm

# Total RAM size of the MCU, in bytes
uint32 total_ram

# Percent of RAM occupied by the system
float32 ram_utilization

# The last message logged from the microcontroller, or empty if a message
# hasn't been logged yet
string message
