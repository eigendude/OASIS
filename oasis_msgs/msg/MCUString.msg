################################################################################
#
#  Copyright (C) 2022-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# This message wraps the primitive string datatype, similar to std_msgs/String.
#
# std_msgs/String was deprecated in Foxy. Programmers are instead encouraged to
# use a more semantically-meaningful message type.
#
# Therefore, this string wrapper is intended for messages logged from
# microcontrollers.
#

# Message header and timestamp
std_msgs/Header header

# The string message
string message
