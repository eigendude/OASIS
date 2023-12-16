################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  This file was derived from the ros_arduino_bridge project under the BSD License.
#  Copyright (C) 2012 Patrick Goebel
#
#  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Reading from a single digital IO pin on an Arduino or any board with GPIO
# connections.
#

# The message header
std_msgs/Header header

# The digital pin number
uint32 digital_pin

# The digital value, specified by LOW or HIGH in AVRConstants.msg
uint8 digital_value
