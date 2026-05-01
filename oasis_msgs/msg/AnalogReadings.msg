################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Readings from multiple analog IO pins on an Arduino or any board with an ADC.
#

# The message header. The timestamp applies to this batch of analog readings.
std_msgs/Header header

# Analog readings
AnalogReading[] readings
