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

UPS_NAME="${UPS_NAME:-ups}"
UPS_DRIVER="${UPS_DRIVER:-usbhid-ups}"
UPS_PORT="${UPS_PORT:-auto}"
UPS_DESC="${UPS_DESC:-Generic USB UPS}"
UPS_VENDOR_ID="${UPS_VENDOR_ID:-0764}"
UPS_PRODUCT_ID="${UPS_PRODUCT_ID:-0501}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UPS_SERVICE_TEMPLATE_SOURCE="${SCRIPT_DIR}/../../oasis_drivers_py/config/systemd/oasis_ups@.service"
UPS_SERVICE_TEMPLATE="/etc/systemd/system/oasis-ups@.service"
UPS_SERVICE_NAME="oasis-ups@${UPS_NAME}.service"

# Config files
NUT_CONF="/etc/nut/nut.conf"
UPS_CONF="/etc/nut/ups.conf"
UPSD_CONF="/etc/nut/upsd.conf"
UPSD_USERS="/etc/nut/upsd.users"
UDEV_RULE="/etc/udev/rules.d/90-nut-${UPS_NAME}.rules"
DRIVER_OVERRIDE_DIR="/etc/systemd/system/nut-driver@.service.d"
SERVER_OVERRIDE_DIR="/etc/systemd/system/nut-server.service.d"

sudo touch "$NUT_CONF" "$UPS_CONF" "$UPSD_CONF" "$UPSD_USERS"

if [[ -f "$UPS_SERVICE_TEMPLATE_SOURCE" ]]; then
  sudo install -m 0644 "$UPS_SERVICE_TEMPLATE_SOURCE" "$UPS_SERVICE_TEMPLATE"
else
  echo "Warning: UPS systemd service template not found at $UPS_SERVICE_TEMPLATE_SOURCE" >&2
fi

# Set mode to standalone (idempotent)
if sudo grep -q "^MODE=" "$NUT_CONF"; then
  sudo sed -i 's/^MODE=.*/MODE=standalone/' "$NUT_CONF"
else
  echo "MODE=standalone" | sudo tee -a "$NUT_CONF" > /dev/null
fi

# Configure the UPS entry
sudo tee "$UPS_CONF" > /dev/null <<EOF_CONF
[${UPS_NAME}]
  driver = ${UPS_DRIVER}
  port = ${UPS_PORT}
  desc = "${UPS_DESC}"
EOF_CONF

# Enable network access for Home Assistant in upsd.conf
if ! sudo grep -q "^LISTEN 0.0.0.0 3493" "$UPSD_CONF" 2>/dev/null; then
  echo "LISTEN 0.0.0.0 3493" | sudo tee -a "$UPSD_CONF" > /dev/null
fi

# Create local NUT user
if ! sudo grep -q "^\[admin\]" "$UPSD_USERS" 2>/dev/null; then
  cat <<EOF_USERS | sudo tee -a "$UPSD_USERS" > /dev/null
[admin]
password = adminpass
actions = SET
instcmds = ALL
EOF_USERS
fi

sudo chown root:nut "$UPSD_USERS"
sudo chmod 640 "$UPSD_USERS"

# Disable legacy always-on services so the driver can be event-driven
sudo systemctl disable --now nut-driver.service || true
sudo systemctl disable --now nut-server.service || true
sudo systemctl disable --now nut-monitor.service || true
sudo systemctl disable --now nut-driver@${UPS_NAME}.service || true
sudo systemctl disable --now "$UPS_SERVICE_NAME" || true

# Ensure any running instances are stopped before reconfiguring them
sudo systemctl stop nut-driver@${UPS_NAME}.service || true
sudo systemctl stop nut-server.service || true
sudo systemctl stop "$UPS_SERVICE_NAME" || true

# Ensure the driver instance behaves like a device-bound service
sudo install -d -m 0755 "$DRIVER_OVERRIDE_DIR"
sudo tee "$DRIVER_OVERRIDE_DIR/override.conf" > /dev/null <<EOF_DRIVER
[Service]
Restart=on-failure
RestartSec=10s
EOF_DRIVER

# Bind the server lifecycle to the driver so it follows the hardware
sudo install -d -m 0755 "$SERVER_OVERRIDE_DIR"
sudo tee "$SERVER_OVERRIDE_DIR/override.conf" > /dev/null <<EOF_SERVER
[Unit]
BindsTo=nut-driver@${UPS_NAME}.service
After=nut-driver@${UPS_NAME}.service
PartOf=nut-driver@${UPS_NAME}.service
EOF_SERVER

# Create a udev rule to start/stop the driver when the UPS is connected/disconnected
sudo tee "$UDEV_RULE" > /dev/null <<EOF_UDEV
# Network UPS Tools event-driven driver for ${UPS_NAME}
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", \
  ATTR{idVendor}=="${UPS_VENDOR_ID}", ATTR{idProduct}=="${UPS_PRODUCT_ID}", \
  TAG+="systemd", ENV{SYSTEMD_WANTS}+="nut-driver@${UPS_NAME}.service", \
  ENV{SYSTEMD_WANTS}+="nut-server.service", \
  ENV{SYSTEMD_WANTS}+="${UPS_SERVICE_NAME}"
ACTION=="remove", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", \
  ATTR{idVendor}=="${UPS_VENDOR_ID}", ATTR{idProduct}=="${UPS_PRODUCT_ID}", \
  RUN+="/bin/systemctl stop nut-driver@${UPS_NAME}.service", \
  RUN+="/bin/systemctl stop nut-server.service", \
  RUN+="/bin/systemctl stop ${UPS_SERVICE_NAME}"
EOF_UDEV

sudo udevadm control --reload || true
sudo systemctl daemon-reload

# If the UPS is already plugged in, trigger udev so the driver starts now
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor="${UPS_VENDOR_ID}" \
  --attr-match=idProduct="${UPS_PRODUCT_ID}" || true
