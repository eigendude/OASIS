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
# A gas concentration measurement
#
# The gas identity and interpretation are supplied by the topic name and
# publisher context. For example, the same message can represent measured CO2,
# equivalent CO2, or total volatile organic compounds.
#

# Measurement timestamp and physical sensor frame
std_msgs/Header header

# Concentration in parts per million
float64 concentration_ppm

# Measurement variance, in ppm^2
#
# A value of 0 indicates that the variance is unknown.
float64 variance
