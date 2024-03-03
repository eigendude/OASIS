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
# Peripheral input
#

# The header and timestamp of the input frame
std_msgs/Header header

# Human-friendly name of the controller as reported by the driver
string name

# Logical address of the peripheral in the system, used to open/close peripherals
string address

# The Kodi controller profile describing the input provided by this peripheral
string controller_profile

# Peripheral input state
DigitalButton[] digital_buttons
AnalogButton[] analog_buttons
AnalogStick[] analog_sticks
Accelerometer[] accelerometers
Throttle[] throttles
Wheel[] wheels
