#!/bin/bash
################################################################################
#
#  Copyright (C) 2025 Garrett Brown
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
# Version parameters
#

NVM_VERSION="0.40.3"

#
# Install dependencies
#

# Install curl and MediaPipe dependencies
sudo apt install -y \
  curl \
  libegl-dev \
  libgles-dev \

# We patch protobuf to match the system version version, so it must be installed
sudo apt install -y \
  libprotobuf-dev \

# We also pull in system glog
sudo apt install -y \
  libgoogle-glog-dev \

# TODO: Needed until we can point MediaPipe at our OpenCV install
sudo apt install -y \
  libopencv-calib3d-dev \
  libopencv-contrib-dev \
  libopencv-core-dev \
  libopencv-features2d-dev \
  libopencv-highgui-dev \
  libopencv-imgproc-dev \
  libopencv-video-dev \

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
