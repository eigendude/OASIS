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
# Install dependencies
#

# Install CMake build dependencies
sudo apt install -y \
  build-essential \
  cmake \
  libssl-dev \
  make \
  tar \
  wget \

# Add ccache support
sudo apt install -y ccache
