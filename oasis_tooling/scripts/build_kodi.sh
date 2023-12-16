#!/bin/bash
################################################################################
#
#  Copyright (C) 2022-2023 Garrett Brown
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

# Import CMake paths and config
source "${SCRIPT_DIR}/env_cmake.sh"

# Import Kodi paths and config
source "${SCRIPT_DIR}/env_kodi.sh"

#
# Load OASIS environment
#

set +o nounset
source "${OASIS_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

#
# Directory setup
#

# Create directories
mkdir -p "${KODI_DOWNLOAD_DIR}"
mkdir -p "${KODI_EXTRACT_DIR}"
mkdir -p "${KODI_BUILD_DIR}"

if [ "${CODENAME}" = "jammy" ]; then
  mkdir -p "${KODI_DEPENDS_DIR}"
fi

#
# Download Kodi
#

if [ ! -f "${KODI_ARCHIVE_PATH}" ]; then
  echo "Downloading Kodi..."
  wget "${KODI_URL}" -O "${KODI_ARCHIVE_PATH}"
fi

#
# Extract Kodi
#

echo "Extracting Kodi..."
tar -zxf "${KODI_ARCHIVE_PATH}" --directory="${KODI_EXTRACT_DIR}"

#
# Patch Kodi
#

echo "Patching Kodi..."
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${KODI_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/kodi/0001-depends-Remove-git-dependency.patch" \
  || :
if [ "${CODENAME}" = "bionic" ]; then
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${KODI_SOURCE_DIR}" \
    < "${CONFIG_DIRECTORY}/kodi/0002-Fix-build-error-on-Ubuntu-18.04-due-to-libnfs-2.0.patch" \
    || :
fi

#
# Configure native depends
#

if [ ! -f "${KODI_DEPENDS_SRC}/Makefile.include" ]; then
  (
    cd "${KODI_DEPENDS_SRC}"
    ./bootstrap
    ./configure \
      --prefix="${KODI_DEPENDS_DIR}" \
      --disable-debug \
      --with-rendersystem=${APP_RENDER_SYSTEM}
  )
fi

#
# Build native depends for target depends
#

make \
  -C "${KODI_DEPENDS_SRC}/native" \
  -j$(getconf _NPROCESSORS_ONLN) \
  meson \
  ninja \
  python3 \

#
# Build libdisplay-info (not available in < Ubuntu 23.04)
#

if [ "${CODENAME}" == "jammy" ]; then
  make \
    -C "${KODI_DEPENDS_SRC}/target/libdisplay-info" \
    -j$(getconf _NPROCESSORS_ONLN)
fi

#
# Configure Kodi
#

(
  echo "Configuring Kodi..."
  mkdir -p "${KODI_BUILD_DIR}"
  cd "${KODI_BUILD_DIR}"
  PATH="${CMAKE_BIN_DIRECTORY}:${PATH}" \
  PKG_CONFIG_PATH="${KODI_DEPENDS_DIR}/${KODI_DEPENDS_TARGET}/lib/pkgconfig" \
    cmake \
      "${KODI_SOURCE_DIR}" \
      -DAPP_RENDER_SYSTEM=${APP_RENDER_SYSTEM} \
      -DCORE_PLATFORM_NAME="x11 gbm$([ "${ENABLE_WAYLAND}" = "OFF" ] || echo " wayland")" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${KODI_INSTALL_DIR}" \
      -DENABLE_CEC=OFF \
      -DENABLE_INTERNAL_FLATBUFFERS=ON \
      -DENABLE_INTERNAL_FMT=${ENABLE_INTERNAL_FMT} \
      -DENABLE_INTERNAL_SPDLOG=${ENABLE_INTERNAL_SPDLOG} \
      -DENABLE_LLD=${ENABLE_LLD} \
      -DENABLE_TESTING=OFF \
)

#
# Build Kodi
#

echo "Building Kodi..."
make \
  -C "${KODI_BUILD_DIR}" \
  -j$(getconf _NPROCESSORS_ONLN) \

#
# Install Kodi
#

echo "Installing Kodi..."
make -C "${KODI_BUILD_DIR}" install

#
# Build native depends for add-ons
#

make \
  -C "${KODI_DEPENDS_SRC}/native" \
  -j$(getconf _NPROCESSORS_ONLN) \
  automake \
  cmake \
  libtool \

#
# Build add-ons
#

echo "Building add-ons..."
make \
  -C "${KODI_DEPENDS_SRC}/target/binary-addons" \
  -j$(getconf _NPROCESSORS_ONLN) \
  ADDONS="peripheral.joystick" \
  PREFIX="${KODI_INSTALL_DIR}" \
