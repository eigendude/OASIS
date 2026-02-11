#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
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

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

#
# Load OASIS dependency environment
#

set +o nounset
source "${OASIS_DEPENDS_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

#
# Install OASIS runtime dependencies
#

# Refresh package metadata
sudo apt update

# Packages to install via apt
APT_PACKAGES=(
  # Needed for resolving hostnames for WoL
  avahi-daemon
  avahi-utils

  # Python dependencies
  python3-psutil
  python3-serial

  # WiFi and network management
  network-manager

  # Useful linux tools
  bmon
  htop

  # Python development tools
  black
  flake8
  isort
  mypy
  tox

  # I2C debugging tools
  i2c-tools
)

# Install display controllers
if [[ ${PLATFORM_ARCH} == i*86 ]] || [[ ${PLATFORM_ARCH} == x86_64 ]]; then
  APT_PACKAGES+=(
    cec-utils
    ddcutil
    vbetool
  )
fi

sudo apt install -y --no-install-recommends "${APT_PACKAGES[@]}"

# Packages to install via pip
PYTHON_PACKAGES=(
  pip \
  psutil \
  git+https://github.com/eigendude/pymata-express@master#egg=pymata-express \
  pyserial \
  setuptools \
  telemetrix-aio \
  tox \
)

# If running on ARM, the latest MediaPipe is v0.10.18
if [[ ${PLATFORM_ARCH} == arm* ]] || [[ ${PLATFORM_ARCH} == aarch64 ]]; then
  PYTHON_PACKAGES+=(
    mediapipe==0.10.18
  )
else
  PYTHON_PACKAGES+=(
    mediapipe==0.10.21
  )
fi

# Install Matplotlib from pip to ensure Mediapipe's 3-D tooling has the newer
# wheel even when an older apt package is already present on the system.
PYTHON_PACKAGES+=(
  matplotlib
)

sudo python3 -m pip install \
  --upgrade \
  --ignore-installed \
  --break-system-packages \
  "${PYTHON_PACKAGES[@]}"
