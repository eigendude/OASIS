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

NUMPY_PYTHON_BIN="${NUMPY_PYTHON_BIN:-python3}"

check_numpy() {
  "${NUMPY_PYTHON_BIN}" -c \
    'import numpy; print(numpy.__version__, numpy.__file__)'
}

install_numpy_without_simd() {
  echo "Installing NumPy without CPU SIMD dispatch..."

  sudo env PIP_ROOT_USER_ACTION=ignore \
    "${NUMPY_PYTHON_BIN}" -m pip install \
    --force-reinstall \
    --break-system-packages \
    --no-binary=numpy \
    --no-cache-dir \
    -Csetup-args=-Dcpu-baseline=none \
    -Csetup-args=-Dcpu-dispatch=none \
    numpy
}

echo "Checking NumPy..."

if check_numpy; then
  echo "Using existing NumPy installation."
  exit 0
else
  numpy_status=$?
fi

if [[ "${numpy_status}" -eq 132 ]] &&
  [[ "$(uname -m)" == "x86_64" ]]
then
  echo "NumPy failed with SIGILL on x86-64."
  install_numpy_without_simd
else
  echo "Installing NumPy..."

  sudo env PIP_ROOT_USER_ACTION=ignore \
    "${NUMPY_PYTHON_BIN}" -m pip install \
    --break-system-packages \
    numpy

  if check_numpy; then
    exit 0
  else
    numpy_status=$?
  fi

  if [[ "${numpy_status}" -eq 132 ]] &&
    [[ "$(uname -m)" == "x86_64" ]]
  then
    echo "Installed NumPy failed with SIGILL on x86-64."
    install_numpy_without_simd
  else
    echo "NumPy import failed with status ${numpy_status}." >&2
    exit "${numpy_status}"
  fi
fi

check_numpy
