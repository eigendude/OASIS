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

# Create the GPIO access group if it does not already exist. OASIS installs a
# udev rule that maps /dev/gpiochip* to root:gpio with mode 0660 for the
# BNO086 interrupt path.
sudo groupadd -f gpio

# Grant the user permission to work with GPIO devices
sudo usermod -a -G gpio ${USER}

################################################################################
# Disable and mask ModemManager to prevent it from grabbing serial interfaces
# needed by AVR programming tools such as avrdude. Masking ensures it cannot be
# started manually or via dependencies.
################################################################################

sudo systemctl disable --now ModemManager.service
sudo systemctl mask ModemManager.service

################################################################################
# Disable the wait-online service to prevent the system from waiting on a
# network connection and prevent the service from starting if requested by
# another service
################################################################################

sudo systemctl disable systemd-networkd-wait-online.service
sudo systemctl mask systemd-networkd-wait-online.service

echo
echo "If you install new OASIS udev rules, reload them with:"
echo "  sudo udevadm control --reload-rules"
echo "  sudo udevadm trigger"
echo
echo "After group membership changes, relogin or reboot before running OASIS"
