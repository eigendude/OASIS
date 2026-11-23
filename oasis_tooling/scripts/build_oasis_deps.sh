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

# Import CMake paths and config
source "${SCRIPT_DIR}/env_cmake.sh"

# Import OpenCV paths and config
source "${SCRIPT_DIR}/env_cv.sh"

# Import OASIS dependency paths and config
source "${SCRIPT_DIR}/env_oasis_deps.sh"

#
# Load ROS 2 environment
#

# Load ROS 2 Desktop
set +o nounset
source "${ROS2_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

#
# Build configuration
#

# Default values
MAKE_FLAGS="${MAKEFLAGS:-}"
COLCON_FLAGS="--merge-install"

if [[ -n "${CMAKE_BUILD_PARALLEL_LEVEL:-}" ]]; then
  MAKE_FLAGS+=" -j${CMAKE_BUILD_PARALLEL_LEVEL}"
fi

# Convert the POSIX-style colon delimited prefix path that we maintain in the
# environment into the semicolon delimited list that CMake expects on the
# command line. This ensures hints like our custom OpenCV toolchain are honored
# even after sourcing the ROS setup file which appends additional prefixes.
cmake_prefix_path_cmake="${CMAKE_PREFIX_PATH//:/;}"

# Add ccache support and fix locating Python
# Also force CMP0074 NEW so find_package() respects *_ROOT hints (e.g., OpenCV_ROOT)
COLCON_FLAGS+=" \
  --cmake-args \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF \
    -DBUILD_PANGOLIN_OPENNI2=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_TESTS=OFF \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    -DCMAKE_POLICY_DEFAULT_CMP0074=NEW \
    -DCMAKE_PREFIX_PATH=${cmake_prefix_path_cmake} \
    -DOpenCV_DIR=${OpenCV_DIR} \
    -DOpenCV_ROOT=${OpenCV_ROOT} \
"

# macOS flags
if [[ "${OSTYPE}" == "darwin"* ]]; then
  COLCON_FLAGS+=" \
    -DCMAKE_INSTALL_RPATH=${OASIS_DEPENDS_INSTALL_DIRECTORY}/lib;${ROS2_INSTALL_DIRECTORY}/lib;${OPENCV_INSTALL_DIR}/lib \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
    -UBoost_DIR -Uboost_*_DIR \
    ${ROS2_PYTHON_CMAKE_ARGS} \
    -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir) \
  "

  # Report independent package failures in the same build pass
  COLCON_FLAGS+=" --continue-on-error"
fi

# Uncomment these to force building in serial
#MAKE_FLAGS+=" -j1 -l1"
#COLCON_FLAGS+=" --executor sequential --event-handlers console_direct+"

#
# Build dependencies with colcon
#

(
  cd "${OASIS_DEPENDS_DIRECTORY}"

  MAKEFLAGS="${MAKE_FLAGS}" \
    colcon build \
      ${COLCON_FLAGS}
)
