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
# Helper script to connect to a WiFi network using nmcli.
#
# Usage: wifi_connect.sh <SSID>
#
# The password will be prompted securely.
#

# Enable strict shell mode
set -o errexit
set -o pipefail
set -o nounset

# Check for required argument
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <SSID>"
    exit 1
fi

SSID="$1"

# Connect with --ask
sudo nmcli device wifi connect "$SSID" --ask

# Log success
echo "Connected to WiFi network '$SSID'"
