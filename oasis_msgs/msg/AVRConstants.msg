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
# AVR-style constants
#

# A digital pin is below the digital threshold
uint8 LOW=0

# A digital pin is avove the digital threshold
uint8 HIGH=1

# An analog pin is disabled
uint8 ANALOG_DISABLED=0

# An analog pin is in input mode
uint8 ANALOG_INPUT=1

# A digital pin is disabled
uint8 DIGITAL_DISABLED=0

# A digital pin is in input mode
uint8 DIGITAL_INPUT=1

# A digital pin is in input mode, with a pullup resistor enabled
uint8 DIGITAL_INPUT_PULLUP=2

# A digital pin is in digital output mode
uint8 DIGITAL_OUTPUT=3

# A digital pin is in PWM output mode
uint8 DIGITAL_PWM=4

# A digital pin is in servo output mode
uint8 DIGITAL_SERVO=5

# A digital pin is in CPU fan control mode
uint8 DIGITAL_CPU_FAN_PWM=6

# A digital pin is in CPU fan speed sensing mode
uint8 DIGITAL_CPU_FAN_TACHOMETER=7
