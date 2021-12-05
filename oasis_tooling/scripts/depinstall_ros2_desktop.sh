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
source "${SCRIPT_DIR}/env_ros2_desktop.sh"

#
# Install dependencies
#

# Install required dependencies
sudo apt install -y curl gnupg2 lsb-release make python3-pip

# Add the ROS 2 apt repository
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Add the ROS 2 repository to our sources list
# NOTE: hersute packages not currently available
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

sudo apt update

# Install development tools and ROS tools
sudo apt install -y build-essential cmake git gfortran libbullet-dev python3-colcon-common-extensions python3-flake8 python3-pip python3-pytest-cov python3-rosdep python3-setuptools python3-vcstool wget

# Install some pip packages needed for testing
python3 -m pip install -U argcomplete flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest

# Install Fast-RTPS dependencies
sudo apt install -y --no-install-recommends libasio-dev libtinyxml2-dev

# Install Cyclone DDS dependencies
sudo apt install -y --no-install-recommends libcunit1-dev

#
# Download the ROS 2 source code
#

echo "Downloading ROS 2 source code..."
(
  cd "${ROS2_SOURCE_DIRECTORY}"

  # Get ROS 2 sources
  wget --timestamping "https://raw.githubusercontent.com/ros2/ros2/${ROS2_DISTRO}/ros2.repos"

  # Patch ROS 2 sources
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    < "${CONFIG_DIRECTORY}/ros2_desktop/0001-Change-rcpputils-to-master-branch.patch"

  # Import ROS 2 sources
  vcs import "${ROS2_SOURCE_DIRECTORY}" < ros2.repos
)

#
# Install rosdep packages
#

# Install rosdep
[ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ] || sudo rosdep init

# Update rosdep
rosdep update

# Install rosdep packages
echo "Installing rosdep packages..."
rosdep install \
  --from-paths "${ROS2_SOURCE_DIRECTORY}" \
  --ignore-src \
  --rosdistro ${ROS2_DISTRO} \
  -y \
  --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

# Add ccache support
dpkg -s ccache >/dev/null || sudo apt install -y ccache
