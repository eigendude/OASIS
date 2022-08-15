#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2022 Garrett Brown
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

# Import CMake paths and config
source "${SCRIPT_DIR}/env_cmake.sh"

# Import ROS 1 paths and config
source "${SCRIPT_DIR}/env_ros1_desktop.sh"

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
#COLCON_FLAGS+=" --executor sequential --event-handlers console_direct+"

#
# Build ROS 1 with colcon
#

echo "Building ROS 1..."

cd "${ROS1_DESKTOP_DIRECTORY}"

MAKE_FLAGS="${MAKE_FLAGS}" \
  colcon build \
    ${COLCON_FLAGS}
