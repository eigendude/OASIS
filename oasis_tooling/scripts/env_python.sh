#!/bin/bash
################################################################################
#
#  Copyright (C) 2022 Garrett Brown
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
# Python configuration
#

# Version
PYTHON_VERSION="3.10.4"

PYTHON_VERSION_MAJOR="$(echo "${PYTHON_VERSION}" | cut -d "." -f1)"
PYTHON_VERSION_MINOR="$(echo "${PYTHON_VERSION}" | cut -d "." -f2)"

# URL
PYTHON_URL="https://www.python.org/ftp/python/${PYTHON_VERSION}/Python-${PYTHON_VERSION}.tgz"

# Installed library name
PYTHON_LIBRARY_NAME="libpython${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}d.so"

#
# Directory and path definitions
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import common paths and config
source "${SCRIPT_DIR}/env_common.sh"

# Subdirectory for Python build files
PYTHON_DIRECTORY="${BUILD_DIRECTORY}/python"

# Define top-level directories for Python
PYTHON_DOWNLOAD_DIR="${PYTHON_DIRECTORY}/downloads"
PYTHON_EXTRACT_DIR="${PYTHON_DIRECTORY}/src"
PYTHON_SOURCE_DIR="${PYTHON_DIRECTORY}/src/Python-${PYTHON_VERSION}"
PYTHON_INSTALL_DIR="${PYTHON_DIRECTORY}/install"

# Installed Python binaries (for external use of Python)
PYTHON_BIN_DIRECTORY="${PYTHON_INSTALL_DIR}/bin"
PYTHON_LIB_DIRECTORY="${PYTHON_INSTALL_DIR}/lib"

# Add Python binaries to system paths
export PATH="${PYTHON_BIN_DIRECTORY}:${PATH}"
export LD_LIBRARY_PATH="${PYTHON_LIB_DIRECTORY}"

# Installed Python site-packages
PYTHON_PKG_DIRECTORY="${PYTHON_INSTALL_DIR}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages"

# Define paths
PYTHON_ARCHIVE_PATH="${PYTHON_DOWNLOAD_DIR}/Python-${PYTHON_VERSION}.tgz"
PYTHON_MAKEFILE_PATH="${PYTHON_SOURCE_DIR}/Makefile"

# Create directories
mkdir -p "${PYTHON_DOWNLOAD_DIR}"
mkdir -p "${PYTHON_EXTRACT_DIR}"

# Locate Python paths
PYTHON_EXECUTABLE="$(which python3)"
PYTHON_INCLUDE_DIR="$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")"
PYTHON_LIBRARY_DIR="$(python3 -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))")"
PYTHON_LIBRARY_PATH="${PYTHON_LIBRARY_DIR}/${PYTHON_LIBRARY_NAME}"
