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
# Integration script to build the ROS packages in this repo.
#
# Call via:
#
#   run_ci.sh <task>
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

# Ensure directory exists
mkdir -p "${BUILD_DIRECTORY}"

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
# Load ROS 2 dependency environment
#

set +o nounset
source "${ROS2_DEPEND_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Install project rosdeps
#

function install() {
  # Install rosdep
  [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ] || sudo rosdep init

  # Update rosdep
  rosdep update

  # Install project rosdeps
  rosdep install \
    --from-paths "${REPO_DIRECTORY}" \
    --ignore-src \
    --rosdistro ${ROS2_DISTRO} \
    -y

  # TODO: image_transport needs libtinyxml2-dev indirectly
  dpkg -s libtinyxml2-dev >/dev/null || sudo apt install -y libtinyxml2-dev
}

#
# Build project with colcon
#

# Set to 0 to disable parallel build
BUILD_PARALLEL=1

# Build flags
MAKE_FLAGS=
COLCON_FLAGS=

if [ ${BUILD_PARALLEL} -eq 0 ]; then
  MAKE_FLAGS="-j1 -l1"
  COLCON_FLAGS="--executor sequential"
fi

function build() {
  cd "${REPO_DIRECTORY}"

  PATH="${CMAKE_BIN_DIRECTORY}:${PATH}" \
    MAKE_FLAGS=${MAKE_FLAGS} colcon build ${COLCON_FLAGS}
}

function lint() {
  : # TODO
}

function format() {
  : # TODO
}

function test() {
  : # TODO
}

function clean() {
  : # TODO
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
  lint)
    lint
    ;;
  format)
    format
    ;;
  test)
    test
    ;;
  clean)
    clean
    ;;
  *)
    echo "Invalid task: ${1:-}"
    exit 1
    ;;
  esac
}

# Perform the dispatch
dispatch ${1:-}
