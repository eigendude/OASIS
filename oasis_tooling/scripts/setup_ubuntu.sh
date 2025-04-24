#!/bin/bash
################################################################################
#
#  Copyright (C) 2023-2025 Garrett Brown
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

################################################################################
# Harware access setup
################################################################################

# Grant the user permission to open serial devices
sudo usermod -a -G dialout ${USER}

# Grant the user permission to use cameras
sudo usermod -a -G video ${USER}

# Grant the user permission to open UPS devices
sudo usermod -a -G plugdev ${USER}

# Grant the user permission to work with I2C devices
sudo usermod -a -G i2c ${USER}
