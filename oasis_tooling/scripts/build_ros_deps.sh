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
# Integration script to build ROS packages that OASIS depends on
#
# Dependencies are automatically installed by the script.
#
# Call via:
#
#   build_ros_deps.sh <task>
#
# See the function dispatch() for the available tasks that can be run.
#

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

#
# Environment configuration
#

# Define the ROS distro to use
ROS2_DISTRO=galactic

#
# Environment paths
#

# ROS 2 paths
ROS2_ROOT=/opt/ros/${ROS2_DISTRO}

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

# Subdirectory for ROS dependency build files
ROS2_DEPEND_DIRECTORY=${BUILD_DIRECTORY}/ros2-depends-${ROS2_DISTRO}

# Directory for ROS 2 sources
SOURCE_DIRECTORY="${ROS2_DEPEND_DIRECTORY}/src"

# Directory for ROS 2 dependency rep;os
IMAGE_TRANSPORT_PLUGINS_REPO_DIR="${SOURCE_DIRECTORY}/ros-perception/image_transport_plugins"

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

function install() {
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
}

#
# Build dependencies with colcon
#

function build() {
  # Patch packages
  # TODO: This revert is only needed for OpenCV < 3.4 (Ubuntu 18.04 ships with 3.2)
  cd "${IMAGE_TRANSPORT_PLUGINS_REPO_DIR}"
  git revert --no-commit 7ca907277eda51ec2fbb71409bda33d1395d6127

  cd "${ROS2_DEPEND_DIRECTORY}"
  PATH="${CMAKE_BIN_DIRECTORY}:${PATH}" \
    `#MAKEFLAGS="-j1 -l1"` \
    colcon build `#--executor sequential`
}

#
# Dispatch function
#
# This function contains the available tasks. The first argument identifies
# which task to jump to.
#
function dispatch() {
  case ${1:-} in
  install)
    install
    ;;
  build)
    build
    ;;
  *)
    echo "Invalid task: ${1:-}"
    exit 1
    ;;
  esac
}

# Perform the dispatch
dispatch ${1:-}
