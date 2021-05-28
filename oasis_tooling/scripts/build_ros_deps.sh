#!/bin/bash
################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of Oasis - https://github.com/eigendude/oasis
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

#
# Integration script to build ROS packages that Oasis depends on
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

# ROS 2 paths
ROS2_ROOT=/opt/ros/${ROS2_DISTRO}

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Directory of the ROS 2 package
PACKAGE_DIRECTORY=${SCRIPT_DIR}/..

# Directory of the Oasis repo
REPO_DIRECTORY=${PACKAGE_DIRECTORY}/..

# Subdirectory for ROS build files
BUILD_DIRECTORY=${REPO_DIRECTORY}/ros-ws

# Subdirectory for CMAKE build files
CMAKE_DIRECTORY=${BUILD_DIRECTORY}/cmake

# Built CMake binaries
CMAKE_BIN_DIRECTORY=${CMAKE_DIRECTORY}/install/bin

# Subdirectory for ROS 2 Desktop build files
ROS2_DESKTOP_DIRECTORY=${BUILD_DIRECTORY}/ros2-desktop-${ROS2_DISTRO}

# Subdirectory for ROS dependency build files
ROS2_DEPEND_DIRECTORY=${BUILD_DIRECTORY}/ros2-depends-${ROS2_DISTRO}

# Directory for ROS 2 sources
SOURCE_DIRECTORY="${ROS2_DEPEND_DIRECTORY}/src"

# Ensure directory exists
mkdir -p "${SOURCE_DIRECTORY}"

# Exclude build directory from rosdep installs
touch "${BUILD_DIRECTORY}/CATKIN_IGNORE"

# Exclude build directory from Colcon builds
touch "${BUILD_DIRECTORY}/COLCON_IGNORE"

#
# Load ROS 2 environment
#

set +o nounset
for ros_setup in \
  "${ROS2_ROOT}/setup.bash" \
  "${ROS2_DESKTOP_DIRECTORY}/install/setup.bash" \
; do
  if [ -f "${ros_setup}" ]; then
    source "${ros_setup}"
    break
  fi
done
set -o nounset

#
# Download dependency source code
#

vcs import "${SOURCE_DIRECTORY}" < "${PACKAGE_DIRECTORY}/config/depends.repos"

#
# Install dependency rosdeps
#

# Install rosdep
[ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ] || sudo rosdep init

# Update rosdep
rosdep update

# Download dependencies
echo "Downloading dependency source code..."

# Install dependency rosdeps
rosdep install \
  --from-paths "${SOURCE_DIRECTORY}" \
  --ignore-src \
  --rosdistro ${ROS2_DISTRO} \
  -y

# TODO: image_transport needs libtinyxml2-dev indirectly
dpkg -s libtinyxml2-dev >/dev/null || sudo apt install -y libtinyxml2-dev

#
# Build dependencies with colcon
#

(
  cd "${ROS2_DEPEND_DIRECTORY}"
  PATH="${CMAKE_BIN_DIRECTORY}:${PATH}" \
    `#MAKEFLAGS="-j1 -l1"` \
    colcon build `#--executor sequential`
)