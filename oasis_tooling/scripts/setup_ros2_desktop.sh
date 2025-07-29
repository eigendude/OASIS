#!/bin/bash
################################################################################
#
#  Copyright (C) 2024-2025 Garrett Brown
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

# Remove any existing ROS apt sources added by older instructions
sudo rm -f \
  /etc/apt/sources.list.d/ros2-latest.list \
  /etc/apt/sources.list.d/ros2.list
sudo sed -i '/packages\.ros\.org\/ros2/d' /etc/apt/sources.list 2>/dev/null || true
sudo find /etc/apt/sources.list.d -name '*.list' -exec \
  sed -i '/packages\.ros\.org\/ros2/d' {} \; 2>/dev/null || true
sudo rm -f \
  /usr/share/keyrings/ros-archive-keyring.gpg \
  /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg

# Add the ROS 2 repository using the ros2-apt-source package
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  sudo apt update
  sudo apt install -y \
    curl \
    gnupg2

  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F"\"" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  rm -f /tmp/ros2-apt-source.deb
fi

sudo apt update
