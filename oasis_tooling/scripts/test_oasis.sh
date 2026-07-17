#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2026 Garrett Brown
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

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

#
# Load OASIS install environment
#

set +o nounset
source "${OASIS_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

#
# Test OASIS with colcon
#

(
  cd "${OASIS_DIRECTORY}"

  colcon test \
    --merge-install \
    --packages-select \
      oasis_drivers_cpp \
      oasis_perception_cpp \
      oasis_visualization \
    --ctest-args \
      -R '^test_'

  colcon test-result \
    --test-result-base build
)
