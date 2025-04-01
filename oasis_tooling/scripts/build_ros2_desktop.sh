#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
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

# Import ROS 2 paths and config
source "${SCRIPT_DIR}/env_ros2_desktop.sh"

#
# Build configuration
#

# Default values
MAKE_FLAGS=
COLCON_FLAGS="--merge-install"
CMAKE_PREFIX_PATH=

# macOS flags
if [[ "${OSTYPE}" == "darwin"* ]]; then
  COLCON_FLAGS+=" --packages-skip-by-dep python_qt_binding"
  CMAKE_PREFIX_PATH="$(brew --prefix qt@5)/lib/cmake/Qt5"
else
  # Add ccache support and fix locating Python
  COLCON_FLAGS+=" \
    --cmake-args \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
  "

  # Skip Qt dependencies, Shiboken is too old on Ubuntu 18.04
  COLCON_FLAGS+=" \
    --packages-ignore \
      qt_gui_cpp \
      rqt_gui_cpp \
  "
fi

# Uncomment these to force building in serial
#MAKE_FLAGS+=" -j1 -l1"
#COLCON_FLAGS+=" --executor sequential --event-handlers console_direct+"

#
# Build ROS 2 with colcon
#

echo "Building ROS 2..."

(
  cd "${ROS2_DESKTOP_DIRECTORY}"

  MAKE_FLAGS="${MAKE_FLAGS}" \
    CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}" \
    colcon build \
      ${COLCON_FLAGS}
)
