#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
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

# Import CMake paths and config
source "${SCRIPT_DIR}/env_cmake.sh"

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

# Get the Ubuntu codename if running on Ubuntu
if [[ "${OSTYPE}" != "darwin"* ]]; then
  CODENAME="$(source "/etc/os-release" && echo "${UBUNTU_CODENAME}")"
else
  CODENAME=
fi

#
# Load OASIS dependency environment
#

set +o nounset
source "${OASIS_DEPENDS_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

#
# Build configuration
#

# Default values
MAKE_FLAGS=
COLCON_FLAGS="--merge-install"

# Add ccache support and fix locating Python
COLCON_FLAGS+=" \
  --cmake-args \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    -DPYTHON_EXECUTABLE:FILEPATH=${PYTHON_EXECUTABLE} \
    -DPYTHON_INCLUDE_DIR=${PYTHON_INCLUDE_DIR} \
    -DPYTHON_LIBRARY=${PYTHON_LIBRARY_PATH} \
"

# Uncomment these to force building in serial
#MAKE_FLAGS+=" -j1 -l1"
#COLCON_FLAGS+=" --executor sequential --event-handlers console_direct+"

#
# Directory setup
#

# Ensure directories exist
mkdir -p "${OASIS_DIRECTORY}"

# Symlink the source directory
if [ ! -L "${OASIS_SOURCE_DIRECTORY}" ]; then
  rm -rf "${OASIS_SOURCE_DIRECTORY}"
  ln -s "${STACK_DIRECTORY}" "${OASIS_SOURCE_DIRECTORY}"
fi

# Disable oasis_perception on Ubuntu 18.04 (OpenCV is too old for bgslibrary now)
if [ "${CODENAME}" = "bionic" ]; then
  touch "${OASIS_SOURCE_DIRECTORY}/oasis_perception/COLCON_IGNORE"
fi

#
# Build OASIS with colcon
#

(
  cd "${OASIS_DIRECTORY}"

  MAKE_FLAGS="${MAKE_FLAGS}" \
    colcon build \
      ${COLCON_FLAGS}
)
