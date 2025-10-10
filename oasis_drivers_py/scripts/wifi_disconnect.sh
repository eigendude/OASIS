#!/bin/bash
################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
#
# Helper script to disconnect from WiFi networks using nmcli.
#
# Usage: wifi_disconnect.sh [SSID]
#
# When an SSID is provided, only that network is disconnected. When no SSID is
# given, all active WiFi connections are disconnected.
#
# Enable strict shell mode
set -o errexit
set -o pipefail
set -o nounset

# Ensure no more than one argument is provided
if [ "$#" -gt 1 ]; then
    echo "Usage: $0 [SSID]"
    exit 1
fi

# If an SSID is provided, disconnect only that network
if [ "$#" -eq 1 ]; then
    SSID="$1"
    sudo nmcli connection down id "$SSID"
    echo "Disconnected from WiFi network '$SSID'"
    exit 0
fi

# No SSID provided, disconnect all active WiFi connections
mapfile -t ACTIVE_WIFI_CONNECTIONS < <(
    nmcli -t -f NAME,TYPE connection show --active \
        | awk -F: '$2 == "wifi" { print $1 }'
)

if [ "${#ACTIVE_WIFI_CONNECTIONS[@]}" -eq 0 ]; then
    echo "No active WiFi connections to disconnect."
    exit 0
fi

for CONNECTION in "${ACTIVE_WIFI_CONNECTIONS[@]}"; do
    sudo nmcli connection down id "$CONNECTION"
    echo "Disconnected from WiFi network '$CONNECTION'"
done
