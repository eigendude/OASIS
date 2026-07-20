#!/bin/bash
################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
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

# Get the absolute path to this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

#
# Version parameters
#

NVM_VERSION="0.40.3"

#
# Install dependencies
#

# Packages to install via apt
APT_PACKAGES=(
  curl

  # EGL/GLES for MediaPipe GPU/OpenGL paths
  libegl-dev
  libgles-dev

  # OpenCV transitive deps pulled by the broad MediaPipe pose monolith
  libavcodec-dev
  libavdevice-dev
  libavformat-dev
  libavif-dev
  libavutil-dev
  libceres-dev
  libgstreamer-plugins-base1.0-dev
  libgstreamer1.0-dev
  libhdf5-dev
  libleptonica-dev
  libopenblas-dev
  libopenexr-dev
  libswscale-dev
  libtesseract-dev

  # Protobuf tools/runtime used at the OASIS/OpenCV boundary
  protobuf-compiler
  libprotobuf-dev

  # Python tooling used by MediaPipe/Bazel
  python3-dev
  python3-venv
  python3-wheel

  # System glog
  libgoogle-glog-dev
)

sudo apt update
sudo apt install -y --no-install-recommends "${APT_PACKAGES[@]}"

#
# Remove PyPI MediaPipe wheel
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  MEDIAPIPE_PIP_PACKAGES=(
    mediapipe
  )

  sudo python3 -m pip uninstall -y \
    --break-system-packages \
    "${MEDIAPIPE_PIP_PACKAGES[@]}"
fi

# Install or validate the shared NumPy installation.
NUMPY_PYTHON_BIN="python3" \
  "${SCRIPT_DIR}/install_numpy.sh"

# Install NVM
curl -o- "https://raw.githubusercontent.com/nvm-sh/nvm/v${NVM_VERSION}/install.sh" | bash

# Add NVM to path
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" # This loads nvm

# Install Node
nvm install node # "node" is an alias for the latest version

# Update npm
npm install -g npm

# Install Bazelisk
npm install -g @bazel/bazelisk
