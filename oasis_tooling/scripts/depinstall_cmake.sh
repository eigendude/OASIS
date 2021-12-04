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
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import environment
source "${SCRIPT_DIR}/env_cmake.sh"

#
# Install dependencies
#

sudo apt update
sudo apt install -y \
  ccache \
  cmake \
  libssl-dev \
  make \
  tar \
  wget \

# Add ccache support
dpkg -s ccache >/dev/null || sudo apt install -y ccache
