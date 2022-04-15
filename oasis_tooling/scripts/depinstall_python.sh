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

# Enable the sources package in the sources list
# TODO: Sed out the comment in /etc/apt/sources.list
echo "deb-src http://archive.ubuntu.com/ubuntu/ $(lsb_release --codename | cut -f2) main" | \
  sudo tee /etc/apt/sources.list.d/debian-sources.list

# Update the packages index
sudo apt update

# Install build dependencies via apt:
sudo apt build-dep -y python3
sudo apt install -y pkg-config

# Install the following packages to build all optional modules
sudo apt install -y \
  build-essential \
  gdb \
  lcov \
  pkg-config \
  libbz2-dev \
  libffi-dev \
  libgdbm-dev \
  libgdbm-compat-dev \
  liblzma-dev \
  libncurses5-dev \
  libreadline6-dev \
  libsqlite3-dev \
  libssl-dev \
  lzma \
  lzma-dev \
  tk-dev \
  uuid-dev \
  zlib1g-dev

# We'll need to hack in a fix for lsb_release
sudo apt install -y lsb-release

# Add ccache support
dpkg -s ccache >/dev/null || sudo apt install -y ccache
