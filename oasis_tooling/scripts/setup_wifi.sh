#!/bin/bash
################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

#
# Connect to and harden a Wi-Fi network.
#
# Usage: setup_wifi.sh <network>
#
# The network password will be prompted securely by nmcli when required.
#

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

#
# Check arguments
#

if [[ "$#" -ne 1 ]]; then
  echo "Usage: $0 <network>"
  exit 1
fi

NETWORK="$1"
WIFI_INTERFACE="wlan0"

#
# Check dependencies
#

if ! command -v nmcli > /dev/null; then
  echo "Error: nmcli is not installed"
  exit 1
fi

if ! command -v iw > /dev/null; then
  echo "Error: iw is not installed"
  exit 1
fi

if ! nmcli device show "${WIFI_INTERFACE}" > /dev/null 2>&1; then
  echo "Error: Wi-Fi interface ${WIFI_INTERFACE} was not found"
  exit 1
fi

#
# Connect to Wi-Fi
#

echo "Connecting ${WIFI_INTERFACE} to Wi-Fi network '${NETWORK}'"

# nmcli securely prompts for the network password when required.
sudo nmcli --ask device wifi connect "${NETWORK}" ifname "${WIFI_INTERFACE}"

CONNECTION_NAME="$(
  nmcli -g GENERAL.CONNECTION device show "${WIFI_INTERFACE}"
)"

if [[ -z "${CONNECTION_NAME}" || "${CONNECTION_NAME}" == "--" ]]; then
  echo "Error: ${WIFI_INTERFACE} has no active NetworkManager connection"
  exit 1
fi

BSSID="$(
  iw dev "${WIFI_INTERFACE}" link |
    awk '$1 == "Connected" && $2 == "to" { print $3; exit }'
)"

if [[ ! "${BSSID}" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]]; then
  echo "Error: Failed to determine the access point BSSID"
  exit 1
fi

#
# Harden Wi-Fi
#

echo "Hardening NetworkManager connection '${CONNECTION_NAME}'"
echo "Access point BSSID: ${BSSID}"

sudo nmcli connection modify "${CONNECTION_NAME}" \
  connection.autoconnect yes \
  connection.autoconnect-retries 0 \
  802-11-wireless.band a \
  802-11-wireless.bssid "${BSSID}" \
  802-11-wireless.cloned-mac-address permanent \
  802-11-wireless.powersave 2

# Apply power-save configuration immediately.
sudo iw dev "${WIFI_INTERFACE}" set power_save off

#
# Verify configuration
#

echo
echo "Saved Wi-Fi settings:"
nmcli connection show "${CONNECTION_NAME}" |
  grep -E \
    '^(connection.autoconnect|connection.autoconnect-retries|802-11-wireless.band|802-11-wireless.bssid|802-11-wireless.cloned-mac-address|802-11-wireless.powersave):'

echo
iw dev "${WIFI_INTERFACE}" link
iw dev "${WIFI_INTERFACE}" get power_save

echo
echo "Connected to and configured Wi-Fi network '${NETWORK}'"
