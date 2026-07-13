################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# A sensor-defined air-quality index measurement
#
# The index scale is defined by the publisher and must be documented alongside
# the publishing topic. This message is not tied to a particular vendor's AQI
# scale.
#

# Measurement timestamp and physical sensor frame
std_msgs/Header header

# Unitless air-quality index
float64 index

# Measurement variance, in squared index units
#
# A value of 0 indicates that the variance is unknown.
float64 variance
