################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

# Measurement time and meter identity
# header.frame_id is the configured power meter ID
std_msgs/Header header

# DC bus voltage [V]
float64 voltage

# Voltage variance [V^2], or 0 when unknown
float64 voltage_variance

# Signed DC current [A]
# Positive means current flowing in the configured forward direction
float64 current

# Current variance [A^2], or 0 when unknown
float64 current_variance

# Signed DC power [W]
# Positive means power flowing in the configured forward direction
float64 power

# Power variance [W^2], or 0 when unknown
float64 power_variance

# Fast hardware overcurrent indication
bool overcurrent

uint8 STATUS_OK=0
uint8 STATUS_STALE=1
uint8 STATUS_DISCONNECTED=2
uint8 STATUS_ERROR=3

# Measurement validity and device state
uint8 status
