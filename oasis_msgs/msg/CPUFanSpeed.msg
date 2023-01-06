################################################################################
#
#  Copyright (C) 2021 Garrett Brown
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

# Digital pin number
uint32 digital_pin

# The speed of the CPU fan, in RPM
float32 fan_speed_rpm
