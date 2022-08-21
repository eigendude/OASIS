#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2022 Garrett Brown
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
# Load ROS 2 environment
#

set +o nounset
source "${ROS2_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

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
  # Add the ROS 2 repository
  ARCH="$(dpkg --print-architecture)"
  CODENAME="$(source "/etc/os-release" && echo "${UBUNTU_CODENAME}")"
  KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
  PKG_URL="http://packages.ros.org/ros2/ubuntu"
  SIGNED_BY="/usr/share/keyrings/ros-archive-keyring.gpg"
  SOURCES_LIST="/etc/apt/sources.list.d/ros2.list"

  if [ ! -f "${SIGNED_BY}" ] || [ ! -f ${SOURCES_LIST} ]; then
    # Install required dependencies
    sudo apt install -y \
      curl \
      gnupg2 \
      lsb-release

    # Authorize the ROS 2 GPG key with apt
    sudo curl -sSL "${KEY_URL}" -o "${SIGNED_BY}"

    # Add the ROS 2 repository to our sources list
    echo "deb [arch=${ARCH} signed-by=${SIGNED_BY}] ${PKG_URL} ${CODENAME} main" | \
      sudo tee "${SOURCES_LIST}"

    sudo apt update
  fi
else
  CODENAME=
fi

#
# Install build dependencies
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Install development tools and ROS tools
  sudo apt install -y \
    build-essential \
    ccache \
    git \
    python3-pip \

  # python3-rosdep is no longer an Ubuntu package, so install via pip
  sudo python3 -m pip install --upgrade rosdep

  # Install vbetool
  if [[ ${PLATFORM_ARCH} == i*86 ]] || [[ ${PLATFORM_ARCH} == x86_64 ]]; then
    sudo apt install -y vbetool
  fi

  # TODO: image_transport needs libtinyxml2-dev indirectly
  sudo apt install -y libtinyxml2-dev

  # Observed on Ubuntu 22.04 that console_bridge wasn't located by CMake
  sudo apt install -y libconsole-bridge-dev
fi

# Upgraded setuptools may be required for other tools
python3 -m pip install --user --upgrade \
  pip \
  setuptools \

# Install development tools and ROS tools
python3 -m pip install --user --upgrade \
  colcon-common-extensions \
  rosdep \
  vcstool \

# numpy is required for building messages
python3 -m pip install --user --upgrade \
  numpy

# Sometimes lark is missing
python3 -m pip install --user --upgrade \
  importlib-resources \
  lark-parser \

# Install OASIS dependencies
python3 -m pip install --user --upgrade \
  --requirement "${STACK_DIRECTORY}/oasis_drivers_py/requirements.txt"

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
    --from-paths "${STACK_DIRECTORY}" \
    --ignore-src \
    --rosdistro ${ROS2_DISTRO} \
    --as-root=pip:false \
    -y
fi

# Bootstrap the Arduino toolchain
"${STACK_DIRECTORY}/oasis_avr/bootstrap.sh"
