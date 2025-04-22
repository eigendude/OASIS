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

#
# Script to install Home Assistant Core on a Debian-based system.
#

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

################################################################################
# Configuration
################################################################################

# The account for Home Assistant Core is called `homeassistant`
HASS_USERNAME="homeassistant"

# The name of the Home Assistant package
HASS_PACKAGE="oasis_hass"

################################################################################
# Environment paths
################################################################################

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Get path to the package directory
HASS_PACKAGE_DIR="${SCRIPT_DIR}/../../${HASS_PACKAGE}"

# The path to the virtual environment for Home Assistant Core
HASS_VENV_PATH="/srv/homeassistant"

# Directory where systemd services are installed
SYSTEMD_SERVICE_DIRECTORY="/etc/systemd/system/"

################################################################################
# Install dependencies
################################################################################

sudo apt update
sudo apt-get install -y \
  autoconf \
  bluez \
  build-essential \
  ffmpeg \
  libatlas-base-dev \
  libffi-dev \
  libjpeg-dev \
  liblapack-dev \
  liblapack3 \
  libopenjp2-7 \
  libssl-dev \
  libtiff6 \
  libturbojpeg0-dev \
  python3 \
  python3-dev \
  python3-pip \
  python3-venv \
  tzdata \
  zlib1g-dev

################################################################################
# Create an account
################################################################################

# Add an account for Home Assistant Core called `homeassistant`. Since this
# account is only for running Home Assistant Core the extra arguments of `-rm`
# is added to create a system account and create a home directory.
if ! id -u "${HASS_USERNAME}" >/dev/null 2>&1; then
  sudo useradd -rm "${HASS_USERNAME}"
fi

################################################################################
# Create the virtual environment
################################################################################

# Create a directory for the installation of Home Assistant Core and change the
# owner to the `homeassistant` account
sudo mkdir -p "${HASS_VENV_PATH}"
sudo chown "${HASS_USERNAME}:${HASS_USERNAME}" "${HASS_VENV_PATH}"

# Create the virtual environment for Home Assistant Core. This is done as the
# `homeassistant` account.
sudo -u "${HASS_USERNAME}" -H bash <<EOF
cd "${HASS_VENV_PATH}"

# Create a virtual environment for Home Assistant Core if one doesn't exist
if [ ! -d "bin" ]; then
  python3 -m venv .
fi

# Activate the virtual environment
source bin/activate

# Install required Python packages
python3 -m pip install --upgrade \
  pip \
  setuptools \
  wheel

# Install Home Assistant Core
pip3 install --upgrade homeassistant
EOF

################################################################################
# Install MQTT broker
################################################################################

# Install the Mosquitto MQTT broker
sudo apt-get install -y \
  mosquitto \
  mosquitto-clients

# Path to custom Mosquitto configuration
MOSQ_CONF_D="/etc/mosquitto/conf.d"
MOSQ_CUSTOM="${MOSQ_CONF_D}/50-oasis.conf"

# Ensure the conf.d directory exists
sudo mkdir -p "${MOSQ_CONF_D}"

# Write our custom listener file to listen on all interfaces
sudo tee "${MOSQ_CUSTOM}" > /dev/null <<EOF
# Listen on all addresses, default port 1883
listener 1883 0.0.0.0

# Allow anonymous clients
allow_anonymous true
EOF

# Enable the Mosquitto service to start on boot
sudo systemctl enable --now mosquitto

# Reload Mosquitto so it picks up the new config
sudo systemctl restart mosquitto

################################################################################
# Configure Home Assistant
################################################################################

HASS_CFG_DIR="/home/${HASS_USERNAME}/.homeassistant"
HASS_CFG_FILE="${HASS_CFG_DIR}/configuration.yaml"

# Ensure the config dir exists
sudo -u "${HASS_USERNAME}" mkdir -p "${HASS_CFG_DIR}"

# Ensure config files exist
sudo -u "${HASS_USERNAME}" touch "${HASS_CFG_DIR}/automations.yaml"
sudo -u "${HASS_USERNAME}" touch "${HASS_CFG_DIR}/scripts.yaml"
sudo -u "${HASS_USERNAME}" touch "${HASS_CFG_DIR}/scenes.yaml"

#
# Append a YAML block only if the top‑level key is absent
#   $1 = key pattern, e.g. "mqtt:"
#   STDIN = YAML to append verbatim
#
insert_block() {
  local key="$1" # Pattern, e.g. "mqtt_statestream:"

  if sudo -u "${HASS_USERNAME}" grep -qE "^${key}" "${HASS_CFG_FILE}" 2>/dev/null; then
    echo "${key} block already present – skipping"
  else
    echo "Adding ${key} block to configuration.yaml"

    # Prepend a single newline, then the block we get on stdin
    { printf '\n'; cat -; } \
      | sudo -u "${HASS_USERNAME}" tee -a "${HASS_CFG_FILE}" >/dev/null
  fi
}

# Create the configuration.yaml file if it doesn't exist and write the default
# config
insert_block "default_config:" <<'YAML'
# Loads default set of integrations. Do not remove.
default_config:
YAML

# Include themes
insert_block "frontend:" <<'YAML'
frontend:
  themes: !include_dir_merge_named themes
YAML

# Include automations.yaml
insert_block "automation:" <<'YAML'
automation: !include automations.yaml
YAML

# Include scripts.yaml
insert_block "script:" <<'YAML'
script: !include scripts.yaml
YAML

# Include scenes.yaml
insert_block "scene:" <<'YAML'
scene: !include scenes.yaml
YAML

# Set default logging level
insert_block "logger:" <<'YAML'
logger:
  default: info
YAML

################################################################################
# Configure MQTT Statestream (Home Assistant -> MQTT)
################################################################################

# MQTT Statestream for lights and switches
insert_block "mqtt_statestream:" <<'YAML'
mqtt_statestream:
  base_topic: homeassistant/statestream
  publish_timestamps: true
  publish_attributes: true
  include:
    domains:
      - light
      - switch
YAML

################################################################################
# Configure automations (MQTT -> Home Assistant)
################################################################################

TEMPLATE_AUTOMATIONS="${HASS_PACKAGE_DIR}/config/homeassistant/automations.yaml"
HA_AUTOMATIONS="${HASS_CFG_DIR}/automations.yaml"

# Write automations.yaml with our MQTT automations
# TODO: Preserve existing automations.yaml content
echo "Writing Home Assistant automations.yaml..."
sudo -u "${HASS_USERNAME}" tee "${HA_AUTOMATIONS}" > /dev/null < "${TEMPLATE_AUTOMATIONS}"

################################################################################
# Install systemd services
################################################################################

# Process systemd services
for SYSTEMD_SERVICE in "${HASS_PACKAGE_DIR}/config/systemd/"*.service; do
  # Get filename
  FILE_NAME="$(basename -- "${SYSTEMD_SERVICE}")"

  # Install systemd service
  echo "Installing ${FILE_NAME}..."
  sudo install -m 0644 "${SYSTEMD_SERVICE}" "${SYSTEMD_SERVICE_DIRECTORY}"

  # Enable systemd servcie
  if [ ! -L "${SYSTEMD_SERVICE_DIRECTORY}/multi-user.target.wants/${FILE_NAME}" ]; then
    sudo ln -s \
      "${SYSTEMD_SERVICE_DIRECTORY}/${FILE_NAME}" \
      "${SYSTEMD_SERVICE_DIRECTORY}/multi-user.target.wants/${FILE_NAME}"
  fi

  # Make updated service files take effect
  echo "Reloading systemd daemon..."
  sudo systemctl daemon-reload

  # Start systemd service
  echo "Starting ${FILE_NAME}..."
  sudo systemctl restart "${FILE_NAME}"
done

################################################################################
# Done
################################################################################
