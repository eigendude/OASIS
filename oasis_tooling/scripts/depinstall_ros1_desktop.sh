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

# Import ROS 1 paths and configuration
source "${SCRIPT_DIR}/env_ros1_desktop.sh"

#
# Setup ROS sources
#

# Install required dependencies
sudo apt install -y curl

# Add the ROS apt repository key
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Add the ROS repository to our sources list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt update

#
# Install dependencies
#

# Install development tools and ROS tools
sudo apt install -y \
  build-essential

# python3-rosdep is no longer an Ubuntu package, so install via pip
sudo python3 -m pip install --upgrade \
  rosdep

# An older version of setuptools is required
python3 -m pip install --user --upgrade \
  pip \
  setuptools==45.2.0 \

# Install development tools and ROS tools
python3 -m pip install --user --upgrade \
  colcon-common-extensions \
  colcon-ros \
  rosdep \
  rosinstall-generator \
  vcstool \

# Install missing Python packages
python3 -m pip install --user --upgrade \
  empy \
  importlib-metadata \
  importlib-resources \
  lark-parser \
  numpy \
  typing-extensions \

# Add ccache support
dpkg -s ccache >/dev/null || sudo apt install -y ccache

#
# Directory setup
#

# Ensure directory exists
mkdir -p "${ROS1_SOURCE_DIRECTORY}"

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
    < "${CONFIG_DIRECTORY}/vision_opencv/0001-Fix-Boost-Python-not-located-on-Ubuntu-Bionic.patch"
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
  --as-root=pip:false \
  -y \
  --skip-keys "python3-pykdl \
"
