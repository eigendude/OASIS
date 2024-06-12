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
# Environment paths and config
################################################################################

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import common paths and config
source "${SCRIPT_DIR}/env_common.sh"

################################################################################
# Setup ROS 2 sources
################################################################################

KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
PKG_URL="http://packages.ros.org/ros2/ubuntu"
SIGNED_BY="/usr/share/keyrings/ros-archive-keyring.gpg"
SOURCES_LIST="/etc/apt/sources.list.d/ros2.list"

# Add the ROS 2 repository
if [ ! -f "${SIGNED_BY}" ] || [ ! -f ${SOURCES_LIST} ]; then
  # Install required dependencies
  sudo apt install -y \
    curl \
    gnupg2

  # Authorize the ROS 2 GPG key with apt
  sudo curl -sSL "${KEY_URL}" -o "${SIGNED_BY}"

  # Add the ROS 2 repository to our sources list
  echo "deb [arch=${ARCH} signed-by=${SIGNED_BY}] ${PKG_URL} ${CODENAME} main" | \
    sudo tee "${SOURCES_LIST}"

  sudo apt update
fi
