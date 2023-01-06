################################################################################
#
#  Copyright (C) 2022 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# An air quality reading
#

# The I2C device that made the reading, or empty if an I2C device wasn't used
I2CDevice[] i2c_device

# The CO2 gas concentration, in ppb, or 0 if CO2 is not measured
float32 co2_ppb

# The TVOC gas concentration, in ppb, or 0 if TVOC is not measured
float32 tvoc_ppb
