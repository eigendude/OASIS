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

UPS_NAME="ups"
UPS_DRIVER="usbhid-ups"
UPS_PORT="auto"
UPS_DESC="Generic USB UPS"
UPS_VENDOR_ID="0764" # CyberPower default
UPS_PRODUCT_ID="0601" # CyberPower default

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

sudo touch "${NUT_CONF}" "${UPS_CONF}" "${UPSD_CONF}" "${UPSD_USERS}"

# Install optional OASIS companion service
if [[ -f "${UPS_SERVICE_TEMPLATE_SOURCE}" ]]; then
  echo "Installing $(basename "${UPS_SERVICE_TEMPLATE_SOURCE}")"
  sudo install -m 0644 "${UPS_SERVICE_TEMPLATE_SOURCE}" "${UPS_SERVICE_TEMPLATE}"
else
  echo "Error: UPS systemd service template not found at ${UPS_SERVICE_TEMPLATE_SOURCE}"
  exit 1
fi

# Update the User= in the templated unit (underscore or hyphen)
sudo sed -i "s|^[[:space:]]*User=.*|User=$(id -un)|" "${UPS_SERVICE_TEMPLATE}"

# Set mode to standalone (idempotent)
echo "Configuring NUT for standalone mode"
if sudo grep -q "^MODE=" "${NUT_CONF}" 2>/dev/null; then
  sudo sed -i 's/^MODE=.*/MODE=standalone/' "${NUT_CONF}"
else
  echo "MODE=standalone" | sudo tee -a "${NUT_CONF}" > /dev/null
fi

# Configure the UPS entry (idempotent overwrite)
echo "Configuring ups.conf for ${UPS_NAME}"
sudo tee "${UPS_CONF}" > /dev/null <<EOF_CONF
[${UPS_NAME}]
  driver = ${UPS_DRIVER}
  port = ${UPS_PORT}
  desc = "${UPS_DESC}"
EOF_CONF

# Enable network access for Home Assistant in upsd.conf
echo "Enabling network access in upsd.conf"
if ! sudo grep -q "^LISTEN 0.0.0.0 3493" "${UPSD_CONF}" 2>/dev/null; then
  echo "LISTEN 0.0.0.0 3493" | sudo tee -a "${UPSD_CONF}" > /dev/null
fi

# Create local NUT users if missing
echo "Configuring upsd.users"
if ! sudo grep -q "^\[admin\]" "${UPSD_USERS}" 2>/dev/null; then
  cat <<EOF_USERS | sudo tee -a "${UPSD_USERS}" > /dev/null
[admin]
password = adminpass
actions = SET
instcmds = ALL
EOF_USERS
fi

sudo chown root:nut "${UPSD_USERS}"
sudo chmod 640 "${UPSD_USERS}"

###############################################################################
# Systemd policy: disable auto-start from enumerator; rely on udev events
###############################################################################

# Stop anything running now
echo "Disabling existing NUT services"
sudo systemctl stop "nut-driver@${UPS_NAME}.service" nut-server.service nut-driver-enumerator.service 2>/dev/null || true
echo "Disabling nut-driver@${UPS_NAME}.service"
sudo systemctl disable --now "nut-driver@${UPS_NAME}.service" 2>/dev/null || true

# Disable enumerator path (prevents auto-instances from ups.conf when device absent)
echo "Disabling nut-driver-enumerator.path"
sudo systemctl disable --now nut-driver-enumerator.path 2>/dev/null || true
# Also ensure enumerator service is stopped and permanently masked so it cannot
# auto-start the driver instance during boot when no UPS is connected.
echo "Stopping nut-driver-enumerator.service"
sudo systemctl stop nut-driver-enumerator.service 2>/dev/null || true
echo "Masking nut-driver-enumerator.service"
sudo systemctl mask nut-driver-enumerator.service 2>/dev/null || true

# Remove any auto-enable symlink created earlier
echo  "Removing any existing nut-driver@${UPS_NAME}.service symlink"
sudo rm -f "/etc/systemd/system/nut-driver.target.wants/nut-driver@${UPS_NAME}.service" 2>/dev/null || true

# Disable always-on server; we'll have udev start it only when the UPS is present
echo "Disabling nut-server.service"
sudo systemctl disable --now nut-server.service 2>/dev/null || true

# nut-monitor.service is not shipped on some distros; ignore errors
echo "Disabling nut-monitor.service"
sudo systemctl stop nut-monitor.service 2>/dev/null || true || true

# The legacy LSB nut-client.service (a SysV wrapper around upsmon) is still
# enabled by default on many distributions and will repeatedly attempt to start
# even when no UPS is connected. Disable it so that service activation becomes
# fully event-driven via the udev rules below.
echo "Disabling nut-client.service"
sudo systemctl disable --now nut-client.service 2>/dev/null || true

###############################################################################
# Overrides
###############################################################################

# Ensure the driver instance behaves like a device-bound service
echo "Creating systemd overrides"
sudo install -d -m 0755 "${DRIVER_OVERRIDE_DIR}"
sudo tee "${DRIVER_OVERRIDE_DIR}/override.conf" > /dev/null <<'EOF_DRIVER'
[Service]
Restart=on-failure
RestartSec=10s
EOF_DRIVER

# Bind the server lifecycle to the driver so it follows the hardware
echo "Binding nut-server.service lifecycle to nut-driver@${UPS_NAME}.service"
sudo install -d -m 0755 "${SERVER_OVERRIDE_DIR}"
sudo tee "${SERVER_OVERRIDE_DIR}/override.conf" > /dev/null <<EOF_SERVER
[Unit]
BindsTo=nut-driver@${UPS_NAME}.service
After=nut-driver@${UPS_NAME}.service
PartOf=nut-driver@${UPS_NAME}.service
EOF_SERVER

###############################################################################
# udev: start/stop services on device add/remove + set permissions
###############################################################################

# udev: start/stop services on device add/remove + set permissions
echo "Creating udev rule at ${UDEV_RULE}"
sudo tee "${UDEV_RULE}" > /dev/null <<EOF_UDEV
# Network UPS Tools event-driven driver for ${UPS_NAME}

# 1) Match the USB device to tag add/remove events for the UPS
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", \
  ATTR{idVendor}=="${UPS_VENDOR_ID}", ATTR{idProduct}=="${UPS_PRODUCT_ID}", \
  TAG+="systemd"

ACTION=="remove", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", \
  ATTR{idVendor}=="${UPS_VENDOR_ID}", ATTR{idProduct}=="${UPS_PRODUCT_ID}", \
  RUN+="/bin/systemctl stop nut-driver@${UPS_NAME}.service", \
  RUN+="/bin/systemctl stop nut-server.service", \
  RUN+="/bin/systemctl stop ${UPS_SERVICE_NAME}"

# 2) Match the HID character device to grant access and start services
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="${UPS_VENDOR_ID}", ATTRS{idProduct}=="${UPS_PRODUCT_ID}", \
  GROUP="nut", MODE="0660", \
  TAG+="systemd", \
  ENV{SYSTEMD_WANTS}+="nut-driver@${UPS_NAME}.service", \
  ENV{SYSTEMD_WANTS}+="nut-server.service", \
  ENV{SYSTEMD_WANTS}+="${UPS_SERVICE_NAME}"
EOF_UDEV

# Apply rules and units
echo "Reloading udev rules and systemd daemon"
sudo udevadm control --reload-rules || true
sudo systemctl daemon-reload

# If the UPS is already plugged in, trigger udev so the driver/server start now
sudo udevadm trigger --subsystem-match=usb \
  --attr-match=idVendor="${UPS_VENDOR_ID}" \
  --attr-match=idProduct="${UPS_PRODUCT_ID}" || true
