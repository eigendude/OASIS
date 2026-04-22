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
# Install build dependencies (everything but macOS)
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Refresh package metadata
  sudo apt update

  # Packages to install via apt
  APT_PACKAGES=(
    # CMake build dependencies
    build-essential
    cmake
    libssl-dev
    make
    tar
    wget

    # ccache support
    ccache
  )

  sudo apt install -y --no-install-recommends "${APT_PACKAGES[@]}"
fi

#
# Install build dependencies (macOS)
#

if [[ "${OSTYPE}" == "darwin"* ]]; then
  brew update

  # Install build utilities
  brew install \
    ccache \
    cmake \
    wget
fi
