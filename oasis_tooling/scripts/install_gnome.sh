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

sudo apt update
sudo apt install -y \
  gnome-shell \
  gnome-shell-extensions \
  gnome-system-monitor \
  gnome-terminal \

#
# Create directories
#

mkdir -p "${GNOME_EXTENSIONS_DIR}"

#
# Install Gnome extensions
#

for extenion_dir in "${VISUALIZATION_DIR}/extensions/"*; do
  echo

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

echo
echo "Configuring Gnome"

# Get the screen lock idle delay
IDLE_DELAY=$(gsettings get org.gnome.desktop.session idle-delay)

# Strip leading type, so that "uint32 0" becomes "0"
IDLE_DELAY="${IDLE_DELAY#* }"

echo "The current screen lock idle delay is: ${IDLE_DELAY}"

# Check if idle-delay is not 0 and then set it to 0
if [ "${IDLE_DELAY}" != "0" ]; then
  echo " - Setting screen lock idle delay to 0"
  gsettings set org.gnome.desktop.session idle-delay 0
fi

# Get the current sleep settings
SLEEP_TYPE="$(gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type)"
echo "The current sleep-inactive-ac-type is: ${SLEEP_TYPE}"

# Check if sleep setting is not "nothing" and then set it to "nothing"
if [ "${SLEEP_TYPE}" != "'nothing'" ]; then
  echo " - Setting sleep-inactive-ac-type to 'nothing'"
  gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
fi

# Check which theme Gnome is using
THEME="$(gsettings get org.gnome.desktop.interface gtk-theme)"
echo "The current Gnome theme is: ${THEME}"

# Check if theme is not Adwaita-dark and then set it to Adwaita-dark
if [ "${THEME}" != "'Adwaita-dark'" ]; then
  echo " - Setting Gnome theme to Adwaita-dark"
  gsettings set org.gnome.desktop.interface gtk-theme 'Adwaita-dark'
fi

#
# Configure systemd
#

echo
echo "Configuring systemd"

# Check if systemd suspend is enabled
SUSPEND_ENABLED="$(systemctl is-enabled sleep.target || :)"
echo "The current systemd suspend is: ${SUSPEND_ENABLED}"

# Check if systemd suspend is not "masked" and then disable it
if [ "${SUSPEND_ENABLED}" != "masked" ]; then
  echo " - Disabling systemd suspend"
  sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
fi
