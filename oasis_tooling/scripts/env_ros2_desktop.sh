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
# Environment configuration
#

# Define the ROS distro to use
ROS2_DISTRO=galactic

#
# Environment paths
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import paths and config
source "${SCRIPT_DIR}/env_common.sh"

# Directory for ROS 2 Desktop build files
ROS2_DESKTOP_DIRECTORY="${BUILD_DIRECTORY}/ros2-desktop-${ROS2_DISTRO}"

# Directory for OASIS dependency build files
OASIS_DEPENDS_DIRECTORY="${BUILD_DIRECTORY}/oasis-depends-${ROS2_DISTRO}"

# Directory for ROS 2 sources
ROS2_SOURCE_DIRECTORY="${ROS2_DESKTOP_DIRECTORY}/src"

# Directory for OASIS dependency sources
OASIS_SOURCE_DIRECTORY="${OASIS_DEPENDS_DIRECTORY}/src"

# Ensure directory exists
mkdir -p "${ROS2_SOURCE_DIRECTORY}"
mkdir -p "${OASIS_SOURCE_DIRECTORY}"
