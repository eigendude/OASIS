#!/bin/bash
################################################################################
#
#  Copyright (C) 2021 Garrett Brown
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

# Import environment
source "${SCRIPT_DIR}/env_ros1_desktop.sh"

#
# Install dependencies
#

# Install required dependencies
sudo apt install -y curl

# Add the ROS apt repository key
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Add the ROS 2 repository to our sources list
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

sudo apt update

# Install development tools and ROS tools
sudo apt install -y build-essential cmake python3-colcon-common-extensions python3-rosdep python3-rosinstall-generator python3-vcstool

#
# Download the ROS 1 source code
#

echo "Downloading ROS 1 source code..."
(
  cd "${ROS1_SOURCE_DIRECTORY}"

  # Get ROS 1 sources
  rosinstall_generator desktop --rosdistro ${ROS1_DISTRO} --deps --tar > "${ROS1_DISTRO}-desktop.rosinstall"

  # Import ROS 1 sources
  vcs import "${ROS1_SOURCE_DIRECTORY}" < "${ROS1_DISTRO}-desktop.rosinstall"

  # Patch ROS 1 sources
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${ROS1_SOURCE_DIRECTORY}/vision_opencv/cv_bridge" \
    < "${CONFIG_DIRECTORY}/cv_bridge/0001-Fix-Boost-Python-not-located-on-Ubuntu-Bionic.patch"
)

#
# Install rosdep packages
#

# Install rosdep
[ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ] || sudo rosdep init

# Update rosdep
rosdep update

# Install ROS 1 system dependencies
echo "Installing ROS 1 system dependencies..."

rosdep install \
  --from-paths "${ROS1_SOURCE_DIRECTORY}" \
  --ignore-src \
  --rosdistro ${ROS1_DISTRO} \
  -y \
  --skip-keys "python3-pykdl"

# Add ccache support
dpkg -s ccache >/dev/null || sudo apt install -y ccache
