################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

# Timestamp and frame_id
std_msgs/Header header

# Unique identifier for this smart plug
string plug_id

# Current on/off state: true = on, false = off
bool state
