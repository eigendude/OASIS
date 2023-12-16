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

# Import OASIS dependency paths and config
source "${SCRIPT_DIR}/env_oasis_deps.sh"

#
# Directory definitions
#

# Directory for OASIS build files
OASIS_DIRECTORY="${BUILD_DIRECTORY}/oasis-${ROS2_DISTRO}"

# Directory for OASIS sources
OASIS_SOURCE_DIRECTORY="${OASIS_DIRECTORY}/src"

# Directory for OASIS installed files
OASIS_INSTALL_DIRECTORY="${OASIS_DIRECTORY}/install"

# Directory for OASIS installed data files
OASIS_DATA_DIRECTORY="${OASIS_INSTALL_DIRECTORY}/share"
