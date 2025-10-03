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
# Install dependencies
#

# Refresh package metadata
sudo apt update

# Packages to install via apt
APT_PACKAGES=(
  # Core build tooling and compilers
  build-essential
  ccache
  cmake
  gfortran
  git
  make
  nasm
  pkg-config
  tar
  wget

  # Python support is needed for the OpenCV Python bindings that are built during
  # the compilation process
  python3-dev
  python3-numpy
  python3-pip
  python3-setuptools
  python3-wheel

  # Video codecs and media frameworks
  # Note that libgstreamer-plugins-bad1.0-dev depends on libopencv-dev, which we
  # do not want to install since we are building our own OpenCV
  libavcodec-dev
  libavformat-dev
  libavutil-dev
  libgstreamer-plugins-base1.0-dev
  libgstreamer-plugins-good1.0-dev
  libgstreamer1.0-dev
  libswscale-dev

  # Image format libraries
  libavif-dev
  libjpeg-dev
  libopenexr-dev
  libopenjp2-7-dev
  libpng-dev
  libtiff-dev
  libwebp-dev
  zlib1g-dev

  # Useful camera / video I/O backends (often helpful for OpenCV)
  libdc1394-dev
  libv4l-dev

  # Math, optimization, and linear algebra backends
  libceres-dev
  libeigen3-dev
  libhdf5-dev
  liblapack-dev
  liblapacke-dev
  libmetis-dev
  libopenblas-dev
  libsuitesparse-dev

  # Google logging library and command-line flags
  libgflags-dev
  libgoogle-glog-dev

  # Protocol Buffers
  libprotobuf-dev
  protobuf-compiler

  # Text rendering and font support
  libfreetype6-dev
  libharfbuzz-dev

  # Optional modules and extras
  libleptonica-dev
  libtbb-dev
  libtesseract-dev
  libva-dev
)

sudo apt install -y --no-install-recommends "${APT_PACKAGES[@]}"
