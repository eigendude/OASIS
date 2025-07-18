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
# MediaPipe configuration
#

# Version
MEDIAPIPE_VERSION="0.10.26"

# URL
MEDIAPIPE_URL="https://github.com/google-ai-edge/mediapipe/archive/refs/tags/v${MEDIAPIPE_VERSION}.tar.gz"

#
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import common paths and config
source "${SCRIPT_DIR}/env_common.sh"

#
# Directory and path definitions
#

# Subdirectory for MediaPipe build files
MEDIAPIPE_DIRECTORY="${BUILD_DIRECTORY}/mediapipe"

# Define top-level directories
MEDIAPIPE_DOWNLOAD_DIR="${MEDIAPIPE_DIRECTORY}/downloads"
MEDIAPIPE_EXTRACT_DIR="${MEDIAPIPE_DIRECTORY}/src"
MEDIAPIPE_SOURCE_DIR="${MEDIAPIPE_DIRECTORY}/src/mediapipe-${MEDIAPIPE_VERSION}"
MEDIAPIPE_BUILD_DIR="${MEDIAPIPE_SOURCE_DIR}/bazel-bin"
MEDIAPIPE_INSTALL_DIR="${MEDIAPIPE_DIRECTORY}/install"

# Define archive path
MEDIAPIPE_ARCHIVE_PATH="${MEDIAPIPE_DOWNLOAD_DIR}/mediapipe-${MEDIAPIPE_VERSION}.tar.gz"
