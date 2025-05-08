#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
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
MAKE_FLAGS=
COLCON_FLAGS="--merge-install"

# macOS flags
if [[ "${OSTYPE}" == "darwin"* ]]; then
  COLCON_FLAGS+=" --packages-skip-by-dep v4l2_camera"
fi

# Add ccache support and fix locating Python
COLCON_FLAGS+=" \
  --cmake-args \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
"

# Uncomment these to force building in serial
#MAKE_FLAGS+=" -j1 -l1"
#COLCON_FLAGS+=" --executor sequential --event-handlers console_direct+"

#
# Build dependencies with colcon
#

(
  cd "${OASIS_DEPENDS_DIRECTORY}"

  MAKE_FLAGS="${MAKE_FLAGS}" \
    colcon build \
      ${COLCON_FLAGS}
)

#
# Install symlimks to fix link error at runtime
#

if \
  [ ! -L "${OASIS_DEPENDS_LIB_DIRECTORY}/libkinect2_registration.so" ] && \
  [ -f "${OASIS_DEPENDS_LIB_DIRECTORY}/kinect2_registration/libkinect2_registration.so" ] \
; then
  rm -rf "${OASIS_DEPENDS_LIB_DIRECTORY}/libkinect2_registration.so"
  ln -s \
    "${OASIS_DEPENDS_LIB_DIRECTORY}/kinect2_registration/libkinect2_registration.so" \
    "${OASIS_DEPENDS_LIB_DIRECTORY}/libkinect2_registration.so"
fi

for so_file in "${OASIS_DEPENDS_LIB_DIRECTORY}/kinect2_bridge/"*.so; do
  target="${OASIS_DEPENDS_LIB_DIRECTORY}/$(basename "$so_file")"
  if [ ! -L "$target" ] && [ -f "$so_file" ]; then
    rm -rf "$target"
    ln -s "$so_file" "$target"
  fi
done
