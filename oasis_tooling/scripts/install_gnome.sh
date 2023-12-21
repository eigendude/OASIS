#!/bin/bash
################################################################################
#
#  Copyright (C) 2023 Garrett Brown
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
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Path to visuaiization package
VISUALIZATION_DIR="${SCRIPT_DIR}/../../oasis_visualization"

# Path to Gnome extensions directory
GNOME_EXTENSIONS_DIR="${HOME}/.local/share/gnome-shell/extensions"

#
# Install Gnome and accessories
#

sudo apt install -y \
  gnome-shell \
  gnome-shell-extensions \
  gnome-terminal \

#
# Create directories
#

mkdir -p "${GNOME_EXTENSIONS_DIR}"

#
# Install Gnome extensions
#

for extenion_dir in "${VISUALIZATION_DIR}/extensions/"*; do
  # Get extension UUID
  EXTENSION_UUID="$(grep -oP '(?<="uuid": ")[^"]*' "${extenion_dir}/metadata.json")"

  echo "Installing ${EXTENSION_UUID}"

  # Create extension directory
  mkdir -p "${GNOME_EXTENSIONS_DIR}/${EXTENSION_UUID}"

  # Copy all files to extension directory
  cp -r "${extenion_dir}/"* "${GNOME_EXTENSIONS_DIR}/${EXTENSION_UUID}"

  # Enable extension with gnome-extensions utility
  echo "Enabling ${EXTENSION_UUID}"
  gnome-extensions enable "${EXTENSION_UUID}"
done

#
# Configure Gnome
#

# Disable screen lock
echo "Disabling screen lock"
gsettings set org.gnome.desktop.session idle-delay 0

# Disable system suspend
echo "Disabling system suspend"
sudo systemctl mask suspend.target
