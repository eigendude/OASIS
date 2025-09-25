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
source "${SCRIPT_DIR}/env_cv.sh"

# Import CMake environment and config
source "${SCRIPT_DIR}/env_cmake.sh"

#
# Directory setup
#

# Create directories
mkdir -p "${OPENCV_DOWNLOAD_DIR}"
mkdir -p "${OPENCV_INSTALL_DIR}"

#
# Download OpenCV
#

if [ ! -f "${OPENCV_ARCHIVE_PATH}" ]; then
  echo "Downloading OpenCV..."
  wget "${OPENCV_URL}" -O "${OPENCV_ARCHIVE_PATH}"
fi

# Download OpenCV Contrib
if [ ! -f "${OPENCV_CONTRIB_ARCHIVE_PATH}" ]; then
  echo "Downloading OpenCV Contrib..."
  wget "${OPENCV_CONTRIB_URL}" -O "${OPENCV_CONTRIB_ARCHIVE_PATH}"
fi

#
# Extract OpenCV
#

echo "Extracting OpenCV..."
rm -rf "${OPENCV_SOURCE_DIR}"
mkdir -p "${OPENCV_SOURCE_DIR}"
tar -zxf "${OPENCV_ARCHIVE_PATH}" --directory="${OPENCV_EXTRACT_DIR}"

#
# Extract OpenCV Contrib
#

echo "Extracting OpenCV Contrib..."
rm -rf "${OPENCV_CONTRIB_SOURCE_DIR}"
mkdir -p "${OPENCV_CONTRIB_SOURCE_DIR}"
tar -zxf "${OPENCV_CONTRIB_ARCHIVE_PATH}" --directory="${OPENCV_EXTRACT_DIR}"

#
# Patch OpenCV
#

echo "Patching OpenCV..."
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OPENCV_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/opencv/0001-GAPI-Implement-RGBA2Gray-and-GBRA2Gray.patch"

#
# Configure OpenCV
#

echo "Configuring OpenCV..."

# TODO: Configure where cv2 ends up, instead of under the prefix
# -DPYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages

cmake_args=(
  -S "${OPENCV_SOURCE_DIR}"
  -B "${OPENCV_BUILD_DIR}"
  -DBUILD_DOCS=OFF
  -DBUILD_EXAMPLES=OFF
  -DBUILD_opencv_apps=OFF
  -DBUILD_opencv_highgui=OFF
  -DBUILD_opencv_java=OFF
  -DBUILD_opencv_python2=OFF
  -DBUILD_opencv_python3=ON
  -DBUILD_opencv_sfm=ON
  -DBUILD_opencv_viz=OFF
  -DBUILD_PERF_TESTS=OFF
  -DBUILD_PROTOBUF=OFF # Use system libprotobuf
  -DBUILD_SHARED_LIBS=ON
  -DBUILD_TESTS=OFF
  -DCMAKE_BUILD_PARALLEL_LEVEL="$(getconf _NPROCESSORS_ONLN)"
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_PREFIX="${OPENCV_INSTALL_DIR}"
  -DENABLE_CCACHE=ON
  -DOPENCV_ENABLE_NONFREE=ON
  -DOPENCV_EXTRA_MODULES_PATH="${OPENCV_CONTRIB_SOURCE_DIR}/modules"
  -DOPENCV_GENERATE_PKGCONFIG=ON
  -DPROTOBUF_UPDATE_FILES=ON
  -DWITH_FFMPEG=ON
  -DWITH_GSTREAMER=ON
  -DWITH_GTK=OFF
  -DWITH_OPENCL=ON
  -DWITH_QT=OFF
  -DWITH_V4L=ON
)

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
