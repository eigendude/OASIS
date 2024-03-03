################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Reading from a single analog IO pin on an Arduino or any board with an ADC.
#

# The message header
std_msgs/Header header

# Analog pin number (ex. A2 on the Arduino is specified as 2)
uint32 analog_pin

# The reference voltage, in Volts
float64 reference_voltage

# The analog value, in a range from 0.0 to 1.0
float64 analog_value
