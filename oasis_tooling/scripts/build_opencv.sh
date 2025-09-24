#!/bin/bash
################################################################################
#
#  Copyright (C) 2025 Garrett Brown
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

# Import OpenCV environment and config
source "${SCRIPT_DIR}/env_opencv.sh"

#
# Directory setup
#

# Create directories
mkdir -p "${OPENCV_DOWNLOAD_DIR}"
mkdir -p "${OPENCV_EXTRACT_DIR}"
mkdir -p "${OPENCV_INSTALL_DIR}"

#
# Download OpenCV
#

if [ ! -f "${OPENCV_ARCHIVE_PATH}" ]; then
  echo "Downloading OpenCV..."
  wget "${OPENCV_URL}" -O "${OPENCV_ARCHIVE_PATH}"
fi

#
# Extract OpenCV
#

echo "Extracting OpenCV..."
rm -rf "${OPENCV_SOURCE_DIR}"
tar -zxf "${OPENCV_ARCHIVE_PATH}" --directory="${OPENCV_EXTRACT_DIR}"

#
# Configure OpenCV
#

cd "${OPENCV_SOURCE_DIR}"

echo "Configuring OpenCV..."

cmake_args=(
  -S .
  -B "${OPENCV_BUILD_DIR}"
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_PREFIX="${OPENCV_INSTALL_DIR}"
  -DCMAKE_BUILD_PARALLEL_LEVEL="$(getconf _NPROCESSORS_ONLN)"
  -DBUILD_TESTS=OFF
  -DBUILD_PERF_TESTS=OFF
  -DBUILD_DOCS=OFF
  -DBUILD_EXAMPLES=OFF
  -DBUILD_opencv_java=OFF
  -DBUILD_opencv_python2=OFF
  -DBUILD_opencv_python3=OFF
  -DOPENCV_GENERATE_PKGCONFIG=ON
)

if command -v ccache &> /dev/null; then
  cmake_args+=(
    -DCMAKE_C_COMPILER_LAUNCHER=ccache
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
  )
fi

cmake "${cmake_args[@]}"

#
# Build OpenCV
#

echo "Building OpenCV..."
cmake --build "${OPENCV_BUILD_DIR}" --parallel "$(getconf _NPROCESSORS_ONLN)"

#
# Install OpenCV
#

echo "Installing OpenCV..."
cmake --install "${OPENCV_BUILD_DIR}"

echo "OpenCV installed into ${OPENCV_INSTALL_DIR}"
