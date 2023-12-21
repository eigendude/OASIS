#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
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
# ROS 1 configuration
#

# Define the ROS distro to use
ROS1_DISTRO=noetic

#
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import Python paths and config
source "${SCRIPT_DIR}/env_python.sh"

#
# Directory definitions
#

# Directory for ROS 1 Desktop build files
ROS1_DESKTOP_DIRECTORY="${BUILD_DIRECTORY}/ros1-desktop-${ROS1_DISTRO}"

# Directory for ROS 1 sources
ROS1_SOURCE_DIRECTORY="${ROS1_DESKTOP_DIRECTORY}/src"
