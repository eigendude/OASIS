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

# TODO

#
# Build OpenCV
#

# TODO

#
# Install OpenCV
#

# TODO

echo "OpenCV installed into ${OPENCV_INSTALL_DIR}"
