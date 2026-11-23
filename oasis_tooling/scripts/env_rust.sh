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

#
# Rust configuration
#

RUST_TOOLCHAIN="stable"
RUST_PROFILE="minimal"
RUSTUP_URL="https://sh.rustup.rs"

#
# Directory and path definitions
#

RUST_CARGO_HOME="${CARGO_HOME:-${HOME}/.cargo}"
RUST_BIN_DIRECTORY="${RUST_CARGO_HOME}/bin"
RUST_ENV_PATH="${RUST_CARGO_HOME}/env"

#
# System path setup
#

# The environment file is absent before the first installation
if [[ -f "${RUST_ENV_PATH}" ]]; then
  # shellcheck source=/dev/null
  source "${RUST_ENV_PATH}"
fi

# Prefer the user toolchain and avoid duplicates when sourced repeatedly
if [[ "${PATH%%:*}" != "${RUST_BIN_DIRECTORY}" ]]; then
  export PATH="${RUST_BIN_DIRECTORY}:${PATH}"
fi
hash -r
