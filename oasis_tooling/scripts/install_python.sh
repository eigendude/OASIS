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

# Install the workspace virtual environment with Homebrew Python
if [[ "${OSTYPE}" == "darwin"* ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
  brew install --formula python3
  source "${SCRIPT_DIR}/env_python.sh"

  "${PYTHON_BASE_EXECUTABLE}" -m venv "${PYTHON_VENV_DIRECTORY}"
  "${PYTHON_VENV_DIRECTORY}/bin/python3" -m pip install --upgrade pip
fi
