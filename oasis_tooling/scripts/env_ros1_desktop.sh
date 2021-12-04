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
ROS1_DISTRO=noetic

#
# Environment paths
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import paths and config
source "${SCRIPT_DIR}/env_common.sh"

# Directory for ROS 1 Desktop build files
ROS1_DESKTOP_DIRECTORY="${BUILD_DIRECTORY}/ros1-desktop-${ROS1_DISTRO}"

# Directory for ROS 1 sources
ROS1_SOURCE_DIRECTORY="${ROS1_DESKTOP_DIRECTORY}/src"

# Ensure directory exists
mkdir -p "${ROS1_SOURCE_DIRECTORY}"
