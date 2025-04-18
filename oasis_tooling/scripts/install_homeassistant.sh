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
HA_USERNAME="homeassistant"

# The name of the homeassistant package
HA_PACKAGE="oasis_ha"

################################################################################
# Environment paths
################################################################################

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Get path to the package directory
HA_PAKCAGE_DIR="${SCRIPT_DIR}/../../${HA_PACKAGE}"

# The path to the virtual environment for Home Assistant Core
HA_VENV_PATH="/srv/homeassistant"

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
if ! id -u "${HA_USERNAME}" >/dev/null 2>&1; then
  sudo useradd -rm "${HA_USERNAME}"
fi

################################################################################
# Create the virtual environment
################################################################################

# Create a directory for the installation of Home Assistant Core and change the
# owner to the `homeassistant` account
sudo mkdir -p "${HA_VENV_PATH}"
sudo chown "${HA_USERNAME}:${HA_USERNAME}" "${HA_VENV_PATH}"

# Create the virtual environment for Home Assistant Core. This is done as the
# `homeassistant` account.
sudo -u "${HA_USERNAME}" -H bash <<EOF
cd "${HA_VENV_PATH}"

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
# Install systemd services
################################################################################

# Process systemd services
for SYSTEMD_SERVICE in "${HA_PAKCAGE_DIR}/config/systemd/"*.service; do
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
  sudo systemctl start "${FILE_NAME}"
done

################################################################################
# Done
################################################################################
