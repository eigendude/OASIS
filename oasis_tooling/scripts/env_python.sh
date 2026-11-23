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

# Use Homebrew Python on macOS and the system interpreter on Linux
if [[ "${OSTYPE}" == "darwin"* ]]; then
  PYTHON_BASE_EXECUTABLE="$(brew --prefix python3)/bin/python3"
else
  PYTHON_BASE_EXECUTABLE="/usr/bin/python3"
fi
PYTHON_VERSION="$("${PYTHON_BASE_EXECUTABLE}" -I -c 'import sysconfig; print(sysconfig.get_python_version())')"

# Get the absolute path to this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${SCRIPT_DIR}/env_common.sh"

if [[ "${OSTYPE}" == "darwin"* ]]; then
  PYTHON_VENV_DIRECTORY="$(cd "${BUILD_DIRECTORY}" && pwd -P)/python-${PYTHON_VERSION}"

  if [[ -f "${PYTHON_VENV_DIRECTORY}/bin/activate" ]] &&
     [[ "${VIRTUAL_ENV:-}" != "${PYTHON_VENV_DIRECTORY}" ]]; then
    # Activation restores the previous venv's PATH before entering the new one
    # Preserve tool directories added since that venv was activated
    if [[ -n "${VIRTUAL_ENV:-}" && -n "${_OLD_VIRTUAL_PATH:-}" ]]; then
      PREVIOUS_VENV_BIN=":${VIRTUAL_ENV}/bin:"
      _OLD_VIRTUAL_PATH=":${PATH}:"
      _OLD_VIRTUAL_PATH="${_OLD_VIRTUAL_PATH//"${PREVIOUS_VENV_BIN}"/:}"
      _OLD_VIRTUAL_PATH="${_OLD_VIRTUAL_PATH#:}"
      _OLD_VIRTUAL_PATH="${_OLD_VIRTUAL_PATH%:}"
      unset PREVIOUS_VENV_BIN
    fi
    source "${PYTHON_VENV_DIRECTORY}/bin/activate"
  fi
fi
