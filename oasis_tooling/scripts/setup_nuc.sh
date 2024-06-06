#!/bin/bash
################################################################################
#
#  Copyright (C) 2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

#
# This script installs the latest kernel on your system.
#
# It is needed for the NUC because the initial version of Ubuntu 24.04 has a
# kernel bug that disables the display unless the "nomodeset" kernel param
# is used.
#

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

# Function to install dependencies
install_dependencies() {
  echo "Installing dependencies..."
  sudo apt update
  sudo apt install -y software-properties-common
}

# Function to add the mainline PPA and install the mainline tool
install_mainline_tool() {
  echo "Adding the mainline PPA and installing the mainline tool..."
  sudo add-apt-repository -y ppa:cappelikan/ppa
  sudo apt update
  sudo apt install -y mainline
}

# Function to install the latest kernel using the mainline tool
install_latest_68_kernel() {
  echo "Installing the latest kernel..."
  sudo mainline install-latest
  echo "Latest kernel installed. Please reboot your system."
}

# Install dependencies
install_dependencies

# Install the mainline tool
install_mainline_tool

# Install the latest 6.8 kernel
install_latest_68_kernel
