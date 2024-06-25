#!/bin/bash
################################################################################
#
#  Copyright (C) 2024 Garrett Brown
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
# Environment paths and configuration
################################################################################

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Path to visuaiization package
VISUALIZATION_DIR="${SCRIPT_DIR}/../../oasis_visualization"

# Path to Gnome extensions directory
GNOME_EXTENSIONS_DIR="${HOME}/.local/share/gnome-shell/extensions"

# Path to Gnome configuration file
GNOME_CONF_FILE="/etc/gdm3/custom.conf"

# Get Gnome version
GNOME_VERSION="$(gnome-shell --version 2>/dev/null || :)"
if [ -n "${GNOME_VERSION}" ]; then
  echo "Detected Gnome version: ${GNOME_VERSION}"
else
  echo "Gnome is not currently installed"
fi

# Host configuration
ENABLE_EXTENSIONS=1

if [ "${HOSTNAME}" == "vision" ] || \
   [ "${HOSTNAME}" == "workstation" ]; then
  ENABLE_EXTENSIONS=0
fi

################################################################################
# Create directories
################################################################################

mkdir -p "${GNOME_EXTENSIONS_DIR}"
