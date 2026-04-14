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
# Generic effect mode constants for known effect kinds
#
# This centralizes mode constants so new effects do not require a new mode msg

# Unknown or unsupported mode
uint8 UNKNOWN=0

# HELIPAD effect modes
uint8 HELIPAD_DISABLED=1
uint8 HELIPAD_GUIDANCE=2
uint8 HELIPAD_LANDED=3

# LED_THRUSTER effect modes
uint8 LED_THRUSTER_DISABLED=10
uint8 LED_THRUSTER_IDLE=11
uint8 LED_THRUSTER_MOVING=12
