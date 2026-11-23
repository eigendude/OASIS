#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2026 Garrett Brown
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
SCRIPT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

# Import Rust paths and config
source "${SCRIPT_DIR}/env_rust.sh"

# Import CMake paths and config
source "${SCRIPT_DIR}/env_cmake.sh"

# Import ROS 2 paths and config
source "${SCRIPT_DIR}/env_ros2_desktop.sh"

#
# Build configuration
#

# Default values
MAKE_FLAGS="${MAKEFLAGS:-}"
COLCON_FLAGS="--merge-install"

if [[ -n "${CMAKE_BUILD_PARALLEL_LEVEL:-}" ]]; then
  MAKE_FLAGS+=" -j${CMAKE_BUILD_PARALLEL_LEVEL}"
fi

# macOS flags
if [[ "${OSTYPE}" == "darwin"* ]]; then
  if [[ ! -x "${PYTHON_VENV_DIRECTORY}/bin/colcon" ]]; then
    echo "Run depinstall_ros2_desktop.sh to install the macOS build tools" >&2
    exit 1
  fi

  # Keep yaml-0.1 include and library paths valid across Homebrew upgrades
  export PKG_CONFIG_YAML_0_1_PREFIX="$(brew --prefix libyaml)"

  # Refresh FindPkgConfig's cached YAML paths from earlier builds
  COLCON_FLAGS+=" \
    --packages-skip-by-dep python_qt_binding \
    --cmake-args \
      -U*YAML_PKG_CONFIG* \
      -DBUILD_TESTING=OFF \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      -DCMAKE_INSTALL_RPATH=${ROS2_INSTALL_DIRECTORY}/lib \
      -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
      ${ROS2_PYTHON_CMAKE_ARGS} \
      -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir) \
      -DCMAKE_PREFIX_PATH=${ROS2_CMAKE_PREFIX_PATH};$(brew --prefix qt@5)/lib/cmake/Qt5${CMAKE_PREFIX_PATH:+;${CMAKE_PREFIX_PATH}} \
  "

  # macOS packages don't all build, so do a best effort to build as much as
  # possible
  COLCON_FLAGS+=" --continue-on-error"
else
  # Add ccache support and fix locating Python
  COLCON_FLAGS+=" \
    --cmake-args \
      -DBUILD_TESTING=OFF \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      -DCMAKE_PREFIX_PATH=${ROS2_CMAKE_PREFIX_PATH}${CMAKE_PREFIX_PATH:+;${CMAKE_PREFIX_PATH}} \
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

  MAKEFLAGS="${MAKE_FLAGS}" \
    colcon build \
      ${COLCON_FLAGS}
)
