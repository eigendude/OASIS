#!/bin/bash
################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
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
SCRIPT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

# Systemd service directory for template installs
SYSTEMD_SERVICE_DIRECTORY="/etc/systemd/system"

# Read the path to the UPS unit configured by build_oasis.sh
UPS_SERVICE="$(
  source "${SCRIPT_DIR}/env_oasis.sh"
  printf '%s\n' "${OASIS_DATA_DIRECTORY}/oasis_drivers_py/systemd/oasis_ups@.service"
)"

if [[ ! -f "${UPS_SERVICE}" || ! -r "${UPS_SERVICE}" ]]; then
  echo "Missing or unreadable unit: ${UPS_SERVICE}. Run ${SCRIPT_DIR}/build_oasis.sh first." >&2
  exit 1
fi
if grep -Eq '@[A-Z0-9_]+@' "${UPS_SERVICE}"; then
  echo "Unresolved placeholder in ${UPS_SERVICE}. Run ${SCRIPT_DIR}/build_oasis.sh first." >&2
  exit 1
fi

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

#
# Install NUT
#

sudo apt update
sudo apt install -y --no-install-recommends nut

sudo touch \
  "${NUT_CONF}" \
  "${UPS_CONF}" \
  "${UPSD_CONF}" \
  "${UPSD_USERS}"

# Install the configured OASIS UPS companion service
echo "Installing $(basename "${UPS_SERVICE}")"
sudo install -m 0644 "${UPS_SERVICE}" "${SYSTEMD_SERVICE_DIRECTORY}"

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

# Restart services for changes to take effect
sudo systemctl restart nut-server.service

# Enable and start driver and server
sudo systemctl enable --now nut-server.service

# Enable and start the OASIS UPS companion service
sudo systemctl enable --now "oasis_ups@${UPS_NAME}.service"

# Disable the default host-based nut-driver@ service if it exists. Debian-based
# systems ship with a generator that attempts to start a driver whose instance
# name matches the system hostname. OASIS uses a dedicated instance name
# ("${UPS_NAME}") instead, so shut down the stray service to avoid repeated
# start failures in the journal.
SYSTEM_HOSTNAME="$(hostname)"
DEFAULT_HOSTNAME_SERVICE="nut-driver@${SYSTEM_HOSTNAME}.service"
if [[ "${UPS_NAME}" != "${SYSTEM_HOSTNAME}" ]]; then
  sudo systemctl disable --now "${DEFAULT_HOSTNAME_SERVICE}" 2>/dev/null || true
fi

# Disable the monitor unless you're using upsmon for shutdown
sudo systemctl disable --now nut-monitor.service || true
