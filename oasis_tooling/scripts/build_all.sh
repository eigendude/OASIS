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

#
# Build CMake
#

"${SCRIPT_DIR}/depinstall_cmake.sh"
"${SCRIPT_DIR}/build_cmake.sh"

#
# Build ROS 2
#

"${SCRIPT_DIR}/depinstall_ros2_desktop.sh"
"${SCRIPT_DIR}/build_ros2_desktop.sh"

#
# Build OASIS dependencies
#

"${SCRIPT_DIR}/depinstall_oasis_deps.sh"
"${SCRIPT_DIR}/build_oasis_deps.sh"

#
# Build MediaPipe
#

"${SCRIPT_DIR}/depinstall_mediapipe.sh"
"${SCRIPT_DIR}/build_mediapipe.sh"

#
# Build OASIS
#

"${SCRIPT_DIR}/depinstall_oasis.sh"
"${SCRIPT_DIR}/build_oasis.sh"

#
# Build Kodi
#

"${SCRIPT_DIR}/depinstall_kodi.sh"
"${SCRIPT_DIR}/build_kodi.sh"
