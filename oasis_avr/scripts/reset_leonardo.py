#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

import sys
import serial

# Environment parameters
SERIAL_PORT = sys.argv[1]

# Open the Leonardo tty at 1200 baud and then close to reset into programming
# mode
print(f"Opening {SERIAL_PORT} at 1200 baud")
ser = serial.Serial(SERIAL_PORT, 1200)

print(f"Closing {SERIAL_PORT}")
ser.close()

print("Arduino Leonardo is now in reset mode for 8 seconds")
