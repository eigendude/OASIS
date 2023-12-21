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
# Environment paths and configuration
#

CODENAME="$(lsb_release --codename | cut -f2)"

KITWARE_KEY="/usr/share/keyrings/kitware-archive-keyring.gpg"
KITWARE_LIST="/etc/apt/sources.list.d/kitware.list"

#
# Install dependencies
#

# Add Kitware repo on Bionic due to stale CMake (3.10) in Ubuntu repo
if [ "${CODENAME}" = "bionic" ]; then
  # Install signing dependencies
  sudo apt install \
    gpg \
    wget \

  # Obtain the Kitware signing key
  wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
    gpg --dearmor - | \
    sudo tee "${KITWARE_KEY}" >/dev/null

  # Add the repository to the sources list
  echo "deb [signed-by=${KITWARE_KEY}] https://apt.kitware.com/ubuntu/ ${CODENAME} main" | \
    sudo tee "${KITWARE_LIST}" >/dev/null

  # Update repositories
  sudo apt update

  # Install the kitware-archive-keyring package to ensure that the keyring
  # stays up to date as Kitware rotates their keys
  sudo rm "${KITWARE_KEY}"
  sudo apt install -y kitware-archive-keyring
fi

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
