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

# Import CMake paths
source "${SCRIPT_DIR}/env_cmake.sh"

# Directory for image_transport_plugins repo
IMAGE_TRANSPORT_PLUGINS_REPO_DIR="${OASIS_SOURCE_DIRECTORY}/ros-perception/image_transport_plugins"

#
# Load ROS 2 environment
#

# Load ROS 2 Desktop
set +o nounset
source "${ROS2_DESKTOP_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Build configuration
#

# Default values
MAKE_FLAGS=
COLCON_FLAGS="--merge-install"

# macOS flags
if [[ "${OSTYPE}" == "darwin"* ]]; then
  COLCON_FLAGS+=" --packages-skip-by-dep v4l2_camera"
fi

# Add ccache support
COLCON_FLAGS+=" --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"

# Uncomment these to force building in serial
#MAKE_FLAGS+=" -j1 -l1"
#COLCON_FLAGS+=" --executor sequential"

#
# Build dependencies with colcon
#

cd "${OASIS_DEPENDS_DIRECTORY}"

PATH="${CMAKE_BIN_DIRECTORY}:${PATH}" \
  MAKE_FLAGS="${MAKE_FLAGS}" \
  colcon build \
    ${COLCON_FLAGS}

#
# Install symlimk to fix link error at runtime
#

if [ ! -f "${OASIS_DEPENDS_DIRECTORY}/install/lib/libkinect2_registration.so" ]; then
  (
    cd "${OASIS_DEPENDS_DIRECTORY}/install/lib"
    ln -s kinect2_registration/libkinect2_registration.so
  )
fi
