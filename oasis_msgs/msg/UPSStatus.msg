################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
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

# The battery voltage, in V, or NaN if unavailable
float32 battery_voltage

# The battery current, in A, or NaN if unavailable
float32 battery_current

# The battery temperature, in degrees Celsius, or NaN if unavailable
float32 battery_temperature

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

# The battery chemistry/type reported by NUT, or empty if unknown
string battery_type
