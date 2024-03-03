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

SOURCES_LIST="/etc/apt/sources.list.d/debian-sources.list"
CODENAME="$(lsb_release --codename | cut -f2)"
PKG_URL="http://archive.ubuntu.com/ubuntu/"

#
# Install dependencies
#

# Enable the sources package in the sources list
if [ ! -f "${SOURCES_LIST}" ]; then
  # TODO: Sed out the comment in /etc/apt/sources.list
  echo "deb-src ${PKG_URL} ${CODENAME} main" | \
    sudo tee "${SOURCES_LIST}"

  # Update the packages index
  sudo apt update
fi

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

# Add ccache support
sudo apt install -y ccache
