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
# Resolve repository paths
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Repository root (one level above oasis_tooling)
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Systemd service directory for template installs
SYSTEMD_SERVICE_DIRECTORY="/etc/systemd/system"

# UPS companion service template paths
UPS_SERVICE_TEMPLATE_SOURCE="${REPO_ROOT}/oasis_drivers_py/config/systemd/oasis_ups@.service"
UPS_SERVICE_TEMPLATE="${SYSTEMD_SERVICE_DIRECTORY}/$(basename "${UPS_SERVICE_TEMPLATE_SOURCE}")"

#
# NUT setup script for Debian-based systems
#

UPS_NAME="ups"
UPS_DRIVER="usbhid-ups"
UPS_PORT="auto"
UPS_DESC="Generic USB UPS"

# Config files
NUT_CONF="/etc/nut/nut.conf"
UPS_CONF="/etc/nut/ups.conf"
UPSD_CONF="/etc/nut/upsd.conf"
UPSD_USERS="/etc/nut/upsd.users"

sudo touch \
  "${NUT_CONF}" \
  "${UPS_CONF}" \
  "${UPSD_CONF}" \
  "${UPSD_USERS}" \

#
# Install NUT
#

sudo apt update
sudo apt install -y --no-install-recommends \
  nut \

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

# Configure the UPS entry
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
  sudo chown root:nut "${UPSD_USERS}"
  sudo chmod 640 "${UPSD_USERS}"
fi

# Reload systemd units to pick up the new template
sudo systemctl daemon-reload

# Disable any host-named driver/companion instances before bringing up the
# dedicated OASIS instance. Debian-based systems ship with a generator that
# creates a driver whose instance name matches the system hostname. Each active
# driver pulls up the matching oasis_ups@ instance (via BindsTo/PartOf), so make
# sure we shut down the stray pair first to avoid duplicate ROS nodes.
SYSTEM_HOSTNAME="$(hostname)"
if command -v systemd-escape >/dev/null 2>&1; then
  DEFAULT_DRIVER_UNIT="$(systemd-escape --template=nut-driver@.service "${SYSTEM_HOSTNAME}")"
  DEFAULT_COMPANION_UNIT="$(systemd-escape --template=oasis_ups@.service "${SYSTEM_HOSTNAME}")"
else
  DEFAULT_DRIVER_UNIT="nut-driver@${SYSTEM_HOSTNAME}.service"
  DEFAULT_COMPANION_UNIT="oasis_ups@${SYSTEM_HOSTNAME}.service"
fi

if [[ "${UPS_NAME}" != "${SYSTEM_HOSTNAME}" ]]; then
  sudo systemctl disable --now "${DEFAULT_DRIVER_UNIT}" >/dev/null 2>&1 || true
  sudo systemctl disable --now "${DEFAULT_COMPANION_UNIT}" >/dev/null 2>&1 || true
fi

# Restart services for changes to take effect
sudo systemctl restart nut-server.service

# Enable and start driver and server
sudo systemctl enable --now nut-server.service

# Enable and start the OASIS UPS companion service
sudo systemctl enable --now "oasis_ups@${UPS_NAME}.service"

# Disable the monitor unless you're using upsmon for shutdown
sudo systemctl disable --now nut-monitor.service || true
