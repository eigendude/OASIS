#!/bin/bash
################################################################################
#
#  Copyright (C) 2021 Garrett Brown
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
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import environment
source "${SCRIPT_DIR}/env_python.sh"

#
# Download Python
#

if [ ! -f "${PYTHON_ARCHIVE_PATH}" ]; then
  echo "Downloading Python..."
  wget "${PYTHON_URL}" -O "${PYTHON_ARCHIVE_PATH}"
fi

#
# Extract Python
#

if [ ! -d "${PYTHON_SOURCE_DIR}" ]; then
  echo "Extracting Python..."
  tar -zvxf "${PYTHON_ARCHIVE_PATH}" --directory="${PYTHON_EXTRACT_DIR}"
fi

#
# Configure Python
#

if [ ! -f "${PYTHON_MAKEFILE_PATH}" ]; then
  (
    echo "Configuring Python..."
    cd "${PYTHON_SOURCE_DIR}"
    ./configure \
      --prefix="${PYTHON_INSTALL_DIR}" \
      --with-pydebug \
      --enable-shared
  )
fi

#
# Build Python
#
# Takes about 90 minutes on a slow netbook, so grab a coffee.
#

echo "Building Python..."
make -C "${PYTHON_SOURCE_DIR}" -j$(getconf _NPROCESSORS_ONLN)

#
# Install Python
#

echo "Installing Python..."
make -C "${PYTHON_SOURCE_DIR}" install

# Add symlink so that lsb_release works with our custom Python version
if [ ! -f "${PYTHON_PKG_DIRECTORY}/lsb_release.py" ]; then
  ln -s "/usr/share/pyshared/lsb_release.py" "${PYTHON_PKG_DIRECTORY}/lsb_release.py"
fi
