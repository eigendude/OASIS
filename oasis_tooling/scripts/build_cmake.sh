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
source "${SCRIPT_DIR}/env_cmake.sh"

#
# Download CMake
#

if [ ! -f "${CMAKE_ARCHIVE_PATH}" ]; then
  echo "Downloading CMake..."
  wget "${CMAKE_URL}" -O "${CMAKE_ARCHIVE_PATH}"
fi

#
# Extract CMake
#

if [ ! -f "${CMAKE_LISTS_PATH}" ]; then
  echo "Extracting CMake..."
  tar -zxf "${CMAKE_ARCHIVE_PATH}" --directory="${CMAKE_EXTRACT_DIR}"
fi

#
# Configure CMake
#
# Takes about 5 minutes on a slow netbook, 10 minutes on a BeagleBone AI
#

if [ ! -f "${CMAKE_MAKEFILE_PATH}" ]; then
  (
    echo "Configuring CMake..."
    mkdir -p "${CMAKE_BUILD_DIR}"
    cd "${CMAKE_BUILD_DIR}"
    cmake \
      "${CMAKE_SOURCE_DIR}" \
      -DCMAKE_INSTALL_PREFIX="${CMAKE_INSTALL_DIR}" \
      -DCMAKE_BUILD_PARALLEL_LEVEL="$(getconf _NPROCESSORS_ONLN)" \
      $(! command -v ccache &> /dev/null || echo "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache")
  )
fi

#
# Build CMake
#
# Takes about 90 minutes on a slow netbook, so grab a coffee.
#

echo "Building CMake..."
make -C "${CMAKE_BUILD_DIR}" -j$(getconf _NPROCESSORS_ONLN)

#
# Install CMake
#

echo "Installing CMake..."
make -C "${CMAKE_BUILD_DIR}" install
