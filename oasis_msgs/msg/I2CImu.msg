################################################################################
#
#  Copyright (C) 2022-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# An IMU reading from an I2C device
#

# The I2C device that made the IMU reading
I2CDevice i2c_device

# The IMU reading
sensor_msgs/Imu imu
