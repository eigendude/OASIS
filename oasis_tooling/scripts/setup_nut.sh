#!/bin/bash
################################################################################
#
#  Copyright (C) 2025 Garrett Brown
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

#
# NUT setup script for Debian-based systems
#

# Config files
NUT_CONF="/etc/nut/nut.conf"
UPS_CONF="/etc/nut/ups.conf"
UPSD_CONF="/etc/nut/upsd.conf"
UPSD_USERS="/etc/nut/upsd.users"
sudo touch "$NUT_CONF" "$UPS_CONF" "$UPSD_CONF" "$UPSD_USERS"

# Set mode to standalone (idempotent)
if sudo grep -q "^MODE=" "$NUT_CONF"; then
  sudo sed -i 's/^MODE=.*/MODE=standalone/' "$NUT_CONF"
else
  echo "MODE=standalone" | sudo tee -a "$NUT_CONF" > /dev/null
fi

# Basic UPS config (CyberPower via USB)
if ! sudo grep -q "^\#[ups\]" "$UPS_CONF" 2>/dev/null; then
  cat <<EOF | sudo tee -a "$UPS_CONF" > /dev/null
#[ups]
#  driver = usbhid-ups
#  port = auto
#  desc = "Generic USB UPS"
EOF
fi

# Enable network access for Home Assistant in upsd.conf
if ! sudo grep -q "^LISTEN 0.0.0.0 3493" "$UPSD_CONF" 2>/dev/null; then
  echo "LISTEN 0.0.0.0 3493" | sudo tee -a "$UPSD_CONF" > /dev/null
fi

# Create local NUT user
if ! sudo grep -q "^\[admin\]" "$UPSD_USERS" 2>/dev/null; then
  cat <<EOF | sudo tee -a "$UPSD_USERS" > /dev/null
[admin]
password = adminpass
actions = SET
instcmds = ALL
EOF
  sudo chown root:nut "$UPSD_USERS"
  sudo chmod 640 "$UPSD_USERS"
fi

# Restart services for changes to take effect
sudo systemctl restart nut-server.service

# Enable and start driver and server
sudo systemctl enable --now nut-server.service

# Disable the monitor unless you're using upsmon for shutdown
sudo systemctl disable --now nut-monitor.service || true
