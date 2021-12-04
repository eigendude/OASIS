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

#
# Load ROS 2 Desktop environment
#

set +o nounset
source "${ROS2_DESKTOP_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Load OASIS dependency environment
#

set +o nounset
source "${OASIS_DEPENDS_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Build configuration
#

# Default values
MAKE_FLAGS=
COLCON_FLAGS="--merge-install"

# Add ccache support
COLCON_FLAGS+=" --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"

# Uncomment these to force building in serial
#MAKE_FLAGS+=" -j1 -l1"
#COLCON_FLAGS+=" --executor sequential"

#
# Build OASIS with colcon
#

cd "${STACK_DIRECTORY}"

PATH="${CMAKE_BIN_DIRECTORY}:${PATH}" \
  MAKE_FLAGS="${MAKE_FLAGS}" \
  colcon build \
    ${COLCON_FLAGS}
