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
# Helper script to list visible WiFi networks using nmcli.
#

# Enable strict shell mode
set -o errexit
set -o pipefail
set -o nounset

# List WiFi networks
nmcli device wifi list
