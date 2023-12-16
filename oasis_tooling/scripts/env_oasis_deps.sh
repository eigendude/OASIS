#!/bin/bash
################################################################################
#
#  Copyright (C) 2022-2023 Garrett Brown
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

# Import ROS 2 paths and config
source "${SCRIPT_DIR}/env_ros2_desktop.sh"

#
# Directory definitions
#

# Directory for OASIS dependency build files
OASIS_DEPENDS_DIRECTORY="${BUILD_DIRECTORY}/oasis-depends-${ROS2_DISTRO}"

# Directory for OASIS dependency sources
OASIS_DEPENDS_SOURCE_DIRECTORY="${OASIS_DEPENDS_DIRECTORY}/src"

# Directory for OASIS dependency installed files
OASIS_DEPENDS_INSTALL_DIRECTORY="${OASIS_DEPENDS_DIRECTORY}/install"

# Directory for OASIS dependency installed libraries
OASIS_DEPENDS_LIB_DIRECTORY="${OASIS_DEPENDS_INSTALL_DIRECTORY}/lib"

#
# Package definitions
#

# libfreenect2 source directory
LIBFREENECT2_SOURCE_DIRECTORY="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2"

# udev fules for libfreenect2
LIBFREENECT2_UDEV_DIRECTORY="${LIBFREENECT2_SOURCE_DIRECTORY}/platform/linux/udev"
