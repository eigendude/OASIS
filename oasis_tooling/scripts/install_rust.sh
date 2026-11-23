#!/bin/bash
################################################################################
#
#  Copyright (C) 2026 Garrett Brown
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

################################################################################
# Environment configuration
################################################################################

# Install Rust for the user who will run the build
if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this script as your regular user, without sudo." >&2
  exit 1
fi

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import Rust paths and config
source "${SCRIPT_DIR}/env_rust.sh"

################################################################################
# Install dependencies
################################################################################

if [[ "${OSTYPE}" == "darwin"* ]]; then
  brew update
else
  sudo apt update

  sudo apt install -y \
    build-essential \
    ca-certificates \
    curl \

fi

################################################################################
# Install or update Rust
################################################################################

if [[ "${OSTYPE}" == "darwin"* ]]; then
  brew install rust
elif [[ -x "${RUST_BIN_DIRECTORY}/rustup" ]]; then
  echo "Updating Rust ${RUST_TOOLCHAIN}..."
  "${RUST_BIN_DIRECTORY}/rustup" toolchain install \
    "${RUST_TOOLCHAIN}" --profile "${RUST_PROFILE}"
else
  echo "Installing Rust ${RUST_TOOLCHAIN}..."
  curl --proto '=https' --tlsv1.2 -sSf "${RUSTUP_URL}" |
    sh -s -- -y --profile "${RUST_PROFILE}" \
      --default-toolchain "${RUST_TOOLCHAIN}"
fi

# Load Rust into this script and prefer it over the system toolchain
source "${SCRIPT_DIR}/env_rust.sh"

if [[ "${OSTYPE}" != "darwin"* ]]; then
  rustup default "${RUST_TOOLCHAIN}"
fi

################################################################################
# Verify installation
################################################################################

if [[ "${OSTYPE}" != "darwin"* ]]; then
  rustup show active-toolchain
fi
rustc --version
cargo --version

################################################################################
# Done
################################################################################

echo
echo "Rust ${RUST_TOOLCHAIN} has been installed and configured."
echo "Run this command to activate Rust in your current shell:"
printf '  source %q\n' "${SCRIPT_DIR}/env_rust.sh"
