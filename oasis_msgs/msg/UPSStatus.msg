################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

# The UPS manufacturer, e.g. CPS, or empty if unknown
string manufacturer

# The UPS model, e.g. CP1500PFCLCDa, or empty if unknown
string model

# The UPS serial number, e.g. CXXNU7006846, or empty if unknown
string serial_number

# The input voltage, in V
float32 input_voltage

# The output voltage, in V
float32 output_voltage

# The battery charge, in percent
uint32 battery_charge

# The battery runtime, in minutes
uint32 battery_runtime

# The current load, in W
float32 load

# The total load supported, in W
float32 load_total

# The status of the battery, or empty if unknown
string status
