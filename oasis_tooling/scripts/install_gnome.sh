#!/bin/bash
################################################################################
#
#  Copyright (C) 2023-2025 Garrett Brown
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

# Import Gnome paths and config
source "${SCRIPT_DIR}/env_gnome.sh"

################################################################################
# Install Gnome and accessories
################################################################################

sudo apt update
sudo apt install -y \
  gnome-disk-utility \
  gnome-shell \
  gnome-shell-extensions \
  gnome-software \
  gnome-system-monitor \
  gnome-terminal \
  gnome-tweaks \
  nautilus \

################################################################################
# Configure Gnome
################################################################################

echo
echo "Configuring Gnome"

#
# Screen lock idle delay
#

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

#
# Sleep settings
#

# Get the current sleep settings
SLEEP_TYPE="$(gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type)"
echo "The current sleep-inactive-ac-type is: ${SLEEP_TYPE}"

# Check if sleep setting is not "nothing" and then set it to "nothing"
if [ "${SLEEP_TYPE}" != "'nothing'" ]; then
  echo " - Setting sleep-inactive-ac-type to 'nothing'"
  gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
fi

#
# Gnome theme
#

# Check which theme Gnome is using
THEME="$(gsettings get org.gnome.desktop.interface gtk-theme)"
echo "The current Gnome theme is: ${THEME}"

# Check if theme is not Adwaita-dark and then set it to Adwaita-dark
if [ "${THEME}" != "'Adwaita-dark'" ]; then
  echo " - Setting Gnome theme to Adwaita-dark"
  gsettings set org.gnome.desktop.interface gtk-theme 'Adwaita-dark'
fi

#
# Screensaver settings
#

# Disable screen saver
echo "Disabling screen saver"
gsettings set org.gnome.desktop.screensaver lock-enabled false
gsettings set org.gnome.desktop.screensaver idle-activation-enabled false

#
# Desktop background settings
#

# Set background properties
echo "Setting background properties"
gsettings set org.gnome.desktop.background picture-options 'zoom'

#
# Display settings
#

# Prevent display from turning off
echo "Preventing display from turning off"
# Disable screensaver's idle activation
gsettings set org.gnome.desktop.screensaver idle-activation-enabled false
# Prevent the display from dimming
gsettings set org.gnome.settings-daemon.plugins.power idle-dim false
# Prevent the system from automatically going to sleep when on AC power
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
# Prevent the system from automatically going to sleep when on battery power
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'

#
# Notification settings
#

# Turn on do-not-dusturb
gsettings set org.gnome.desktop.notifications show-banners false

#
# Auto-login
#

# Set auto login configuration
LINE1="AutomaticLoginEnable=True"
LINE2="AutomaticLogin=$(whoami)"

# Check if any line appears in ${GNOME_CONF_FILE} and print autologin enabled if so
if grep -qx "${LINE1}" "${GNOME_CONF_FILE}" && \
   grep -qx "${LINE2}" ${GNOME_CONF_FILE}; then
  echo "Autologin is enabled"
else
  echo "Autologin is disabled"
  echo " - Enabling autologin"
fi

for LINE in "${LINE2}" "${LINE1}"; do
  # Check if line is not in the file and then add it
  if ! grep -qx "${LINE}" "${GNOME_CONF_FILE}"; then
    echo " - Adding ${LINE}"
    sudo sed -i "/^\[daemon\]/a ${LINE}" "${GNOME_CONF_FILE}"
  fi
done

#
# Enable extensions
#

echo "Enabling extensions"
gsettings set org.gnome.shell disable-user-extensions false

################################################################################
# Configure system
################################################################################

echo
echo "Configuring system"

#
# Systemd suspend
#

# Check if systemd suspend is enabled
SUSPEND_ENABLED="$(systemctl is-enabled sleep.target || :)"
echo "The current systemd suspend is: ${SUSPEND_ENABLED}"

# Check if systemd suspend is not "masked" and then disable it
if [ "${SUSPEND_ENABLED}" != "masked" ]; then
  echo " - Disabling systemd suspend"
  sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
fi

#
# Disable Apport crash reporting service
#

# Check the current status of Apport
apport_status=$(grep 'enabled=' /etc/default/apport)

if [[ $apport_status == "enabled=1" ]]; then
  echo "Apport is currently: enabled"
  echo " - Disabling Apport"

  # Disable Apport in the configuration file
  sudo sed -i 's/enabled=1/enabled=0/' /etc/default/apport

  # Stop and disable the Apport service
  sudo systemctl stop apport.service
  sudo systemctl disable apport.service
else
  echo "Apport is currently: disabled"
fi

#
# Disable "Updated software" dialog
#

# Disables the Ubuntu auto-update dialog by setting the update values to "0"
UPDATE_CONFIG="/etc/apt/apt.conf.d/20auto-upgrades"
if [ -f "${UPDATE_CONFIG}" ]; then
  # Check if both settings are already set to "0"
  if grep -q 'APT::Periodic::Update-Package-Lists "0"' "${UPDATE_CONFIG}" && \
     grep -q 'APT::Periodic::Unattended-Upgrade "0"' "${UPDATE_CONFIG}"; then
    echo "Auto-update config in ${UPDATE_CONFIG} is already disabled"
  else
    echo "Disabling auto-updates in ${UPDATE_CONFIG}..."
    sudo sed -i -e 's/APT::Periodic::Update-Package-Lists "1"/APT::Periodic::Update-Package-Lists "0"/' \
                -e 's/APT::Periodic::Unattended-Upgrade "1"/APT::Periodic::Unattended-Upgrade "0"/' \
                "${UPDATE_CONFIG}"
    echo "Auto-update dialog has been disabled"
  fi
fi

################################################################################
# Install Gnome extensions
################################################################################

# Check if extensions are configured
if [ "${ENABLE_EXTENSIONS}" -eq 1 ]; then
  echo

  # Need to restart Gnome so that new extensions can be enabled
  echo "Restarting gdm..."
  sleep 1 # Give Gnome a moment to finish up
  sudo systemctl restart gdm

  # Disable window list extension
  gnome-extensions disable window-list@gnome-shell-extensions.gcampax.github.com

  for extension_dir in "${VISUALIZATION_DIR}/extensions/"*; do
    # Skip non-directories (e.g. README.md)
    if [ ! -d "${extension_dir}" ]; then
      continue
    fi

    # Get extension UUID
    EXTENSION_UUID="$(grep -oP '(?<="uuid": ")[^"]*' "${extension_dir}/metadata.json")"

    echo "Installing ${EXTENSION_UUID}"

    # Create extension directory
    mkdir -p "${GNOME_EXTENSIONS_DIR}/${EXTENSION_UUID}"

    # Copy all files to extension directory
    cp -r "${extension_dir}/"* "${GNOME_EXTENSIONS_DIR}/${EXTENSION_UUID}"

    # Enable extension with gnome-extensions utility
    echo " - Enabling ${EXTENSION_UUID}"
    gnome-extensions enable "${EXTENSION_UUID}"
  done
fi

################################################################################
# Disable GNOME Tracker (file indexer)
################################################################################

echo
echo "Disabling GNOME Tracker indexer"

# Helper: run a gsettings write only if the schema/key exists
gset_if_writable() {
  local schema="$1" key="$2" value="$3"
  if gsettings list-schemas | grep -qx "$schema" && gsettings writable "$schema" "$key" >/dev/null 2>&1; then
    echo " - $schema $key -> $value"
    gsettings set "$schema" "$key" $value
  fi
}

# Stop current Tracker daemons if the CLI exists
if command -v tracker3 >/dev/null 2>&1; then
  echo " - Stopping tracker3 daemon"
  tracker3 daemon -t || :
elif command -v tracker >/dev/null 2>&1; then
  echo " - Stopping tracker daemon"
  tracker daemon -t || :
fi

# Tell Tracker not to crawl or watch for changes (Tracker 3 schema)
gset_if_writable org.freedesktop.Tracker3.Miner.Files enable-monitors false
gset_if_writable org.freedesktop.Tracker3.Miner.Files crawling-interval -2
gset_if_writable org.freedesktop.Tracker3.Miner.Files index-recursive-directories "[]"
gset_if_writable org.freedesktop.Tracker3.Miner.Files index-single-directories "[]"

# Fallback for older installations (Tracker 2 schema)
gset_if_writable org.freedesktop.Tracker.Miner.Files enable-monitors false
gset_if_writable org.freedesktop.Tracker.Miner.Files crawling-interval -2
gset_if_writable org.freedesktop.Tracker.Miner.Files index-recursive-directories "[]"
gset_if_writable org.freedesktop.Tracker.Miner.Files index-single-directories "[]"

# Mask user units so they cannot auto-start again (names vary by distro)
TRACKER_UNITS=(
  tracker-miner-fs-3.service
  tracker-extract-3.service
  tracker-miner-apps-3.service
  tracker-miner-rss-3.service
  tracker-writeback-3.service
  tracker3.service
  tracker-store-3.service
)
for unit in "${TRACKER_UNITS[@]}"; do
  if systemctl --user list-unit-files | awk '{print $1}' | grep -qx "$unit"; then
    state="$(systemctl --user is-enabled "$unit" 2>/dev/null || echo "unknown")"
    echo " - $unit is $state; stopping and masking"
    systemctl --user stop "$unit" || :
    systemctl --user mask "$unit" || :
  fi
done

# Also mask corresponding timers if present
for timer in tracker-miner-fs-3.timer tracker-extract-3.timer; do
  if systemctl --user list-unit-files | awk '{print $1}' | grep -qx "$timer"; then
    echo " - Masking $timer"
    systemctl --user stop "$timer" || :
    systemctl --user mask "$timer" || :
  fi
done

# Prevent autostart .desktop launchers (defensive; some builds use them)
mkdir -p "${HOME}/.config/autostart"
for f in /etc/xdg/autostart/tracker-*.desktop; do
  [ -f "$f" ] || continue
  base="$(basename "$f")"
  dest="${HOME}/.config/autostart/${base}"
  if [ ! -f "$dest" ]; then
    echo " - Creating autostart override for ${base}"
    cp "$f" "$dest"
  fi
  # Ensure Hidden=true is present
  if ! grep -q '^Hidden=true' "$dest"; then
    echo "Hidden=true" >> "$dest"
  fi
done

echo "GNOME Tracker has been disabled."

################################################################################
# Done
################################################################################

echo
echo "Gnome \"$(gnome-shell --version)\" has been installed and configured"
