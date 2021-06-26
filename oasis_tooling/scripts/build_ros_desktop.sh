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

#
# Integration script to build a ROS 2 Desktop installation
#
# A check is made to see if ROS 2 can be installed via apt for the current
# distribution, and this script exits if so.
#
# Dependencies are automatically installed by the script.
#

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

#
# Environment configuration
#

# Define the ROS distro to use
ROS2_DISTRO=foxy

#
# Environment paths
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Directory of the ROS 2 package
PACKAGE_DIRECTORY=${SCRIPT_DIR}/..

# Directory of the OASIS repo
REPO_DIRECTORY=${PACKAGE_DIRECTORY}/..

# Subdirectory for ROS build files
BUILD_DIRECTORY=${REPO_DIRECTORY}/ros-ws

# Subdirectory for CMAKE build files
CMAKE_DIRECTORY=${BUILD_DIRECTORY}/cmake

# Built CMake binaries
CMAKE_BIN_DIRECTORY=${CMAKE_DIRECTORY}/install/bin

# Subdirectory for ROS 2 Desktop build files
ROS2_DESKTOP_DIRECTORY=${BUILD_DIRECTORY}/ros2-desktop-${ROS2_DISTRO}

# Directory for ROS 2 sources
SOURCE_DIRECTORY="${ROS2_DESKTOP_DIRECTORY}/src"

# Ensure directory exists
mkdir -p "${SOURCE_DIRECTORY}"

# Exclude build directory from rosdep installs
touch "${BUILD_DIRECTORY}/CATKIN_IGNORE"

# Exclude build directory from Colcon builds
touch "${BUILD_DIRECTORY}/COLCON_IGNORE"

#
# Build configuration
#

# Get current OS distro
DISTRO=$(lsb_release -cs)

# Decide if ROS 2 needs to be compiled
case ${DISTRO} in
  # Ubuntu 18.04
  bionic)
    BUILD_ROS2_DESKTOP=1
    ;;
  # Ubuntu 20.04
  focal)
    BUILD_ROS2_DESKTOP=0
    ;;
  # Ubuntu 21.04
  hirsute)
    BUILD_ROS2_DESKTOP=1
    ;;
  *)
    echo "Unknown distro: ${DISTRO}"
    BUILD_ROS2_DESKTOP=1
    ;;
esac

#
# Check for system ROS 2
#

# If we don't need to build ROS 2, exit now
if [ ${BUILD_ROS2_DESKTOP} -eq 0 ]; then
  echo "Using system ROS 2"
  exit 0
else
  echo "Building ROS 2"
  echo
fi

#
# If ROS 2 is not available from the system, we need to install dependencies
# required to build it.
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
  cd "${SOURCE_DIRECTORY}"
  wget --timestamping "https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos"
  vcs import "${SOURCE_DIRECTORY}" < ros2.repos
)

#
# Install rosdep dependencies
#

# Install rosdep
[ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ] || sudo rosdep init

# Update rosdep
rosdep update

# Install ROS 2 system dependencies
echo "Installing ROS 2 system dependencies..."
rosdep install \
  --from-paths "${SOURCE_DIRECTORY}" \
  --ignore-src \
  --rosdistro ${ROS2_DISTRO} \
  -y \
  --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

#
# Build ROS 2 with colcon
#

echo "Building ROS 2..."
(
  cd "${ROS2_DESKTOP_DIRECTORY}"
  PATH="${CMAKE_BIN_DIRECTORY}:${PATH}" \
    `#MAKEFLAGS="-j1 -l1"` \
    colcon build `#--executor sequential`
)
