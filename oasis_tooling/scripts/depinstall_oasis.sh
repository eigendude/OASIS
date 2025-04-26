#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
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
# Setup ROS 2 sources
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  "${SCRIPT_DIR}/setup_ros2_desktop.sh"
fi

#
# Install build dependencies (everything but macOS)
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Install development tools and ROS tools
  sudo apt install -y --no-install-recommends \
    build-essential \
    ccache \
    cmake \
    git \
    ros-dev-tools \

  # Needed for ROS message generation
  sudo apt install -y --no-install-recommends \
    python3-lark \

  # Needed by image_transport and plugins
  sudo apt install -y --no-install-recommends \
    libconsole-bridge-dev \
    libspdlog-dev \
    libtinyxml2-dev \

  # Install vbetool
  if [[ ${PLATFORM_ARCH} == i*86 ]] || [[ ${PLATFORM_ARCH} == x86_64 ]]; then
    sudo apt install -y vbetool
  fi

  # Install ddcutil
  sudo apt install -y ddcutil

  # Install Python dependencies
  sudo apt install -y --no-install-recommends \
    python3-numpy \
    python3-psutil \
    python3-serial
  sudo python3 -m pip install --upgrade --break-system-packages \
    mediapipe \
    telemetrix-aio

  # Needed for communicating with UPS devices
  sudo apt install -y --no-install-recommends \
    nut

  # Config files
  NUT_CONF="/etc/nut/nut.conf"
  UPS_CONF="/etc/nut/ups.conf"
  UPSD_CONF="/etc/nut/upsd.conf"
  UPSD_USERS="/etc/nut/upsd.users"
  sudo touch "$NUT_CONF" "$UPS_CONF" "$UPSD_CONF" "$UPSD_USERS"

  # Set mode to standalone (idempotent)
  if sudo grep -q "^MODE=" "$NUT_CONF"; then
    sudo sed -i 's/^MODE=.*/MODE=standalone/' "$NUT_CONF"
  else
    echo "MODE=standalone" | sudo tee -a "$NUT_CONF" > /dev/null
  fi

  # Basic UPS config (CyberPower via USB)
  if ! sudo grep -q "^\[ups\]" "$UPS_CONF" 2>/dev/null; then
    cat <<EOF | sudo tee "$UPS_CONF" > /dev/null
[ups]
  driver = usbhid-ups
  port = auto
  desc = "Generic USB UPS"
EOF
  fi

  # Enable network access for Home Assistant in upsd.conf
  if ! sudo grep -q "^LISTEN 0.0.0.0 3493" "$UPSD_CONF" 2>/dev/null; then
    echo "LISTEN 0.0.0.0 3493" | sudo tee -a "$UPSD_CONF" > /dev/null
  fi

  # Create local NUT user
  if ! sudo grep -q "^\[admin\]" "$UPSD_USERS" 2>/dev/null; then
    cat <<EOF | sudo tee "$UPSD_USERS" > /dev/null
[admin]
  password = adminpass
  actions = SET
  instcmds = ALL
EOF
    sudo chown root:nut "$UPSD_USERS"
    sudo chmod 640 "$UPSD_USERS"
  fi

  # Restart services for changes to take effect
  sudo systemctl restart nut-server.service

  # Enable and start driver and server
  sudo systemctl enable --now nut-server.service

  # Disable the monitor unless you're using upsmon for shutdown
  sudo systemctl disable --now nut-monitor.service || true
fi

#
# Install OASIS rosdeps
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Install rosdep
  [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ] || sudo rosdep init

  # Update rosdep
  rosdep update

  # Install OASIS rosdeps
  rosdep install \
    --os=ubuntu:${CODENAME} \
    --from-paths "${STACK_DIRECTORY}" \
    --ignore-src \
    --rosdistro ${ROS2_DISTRO} \
    --as-root=pip:false \
    --default-yes
fi

# Bootstrap the Arduino toolchain
"${STACK_DIRECTORY}/oasis_avr/bootstrap.sh"
