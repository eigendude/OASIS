#!/bin/bash
################################################################################
#
#  Copyright (C) 2012-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

# Detect Arduino
ls -l /dev/serial/by-id 2> /dev/null | grep -i "arduino"

# Output looks like:
# lrwxrwxrwx 1 root root 13 2012-03-03 20:49 usb-Arduino__www.arduino.cc__0042_64938333932351303182-if00 -> ../../ttyACM0
