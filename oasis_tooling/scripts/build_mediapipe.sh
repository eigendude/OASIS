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
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import MediaPipe environment and config
source "${SCRIPT_DIR}/env_mediapipe.sh"

# Add NVM to path for Bazelisk
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" # This loads nvm

#
# Directory setup
#

# Create directories
mkdir -p "${MEDIAPIPE_DOWNLOAD_DIR}"
mkdir -p "${MEDIAPIPE_EXTRACT_DIR}"
mkdir -p "${MEDIAPIPE_INSTALL_DIR}"

#
# Download MediaPipe
#

if [ ! -f "${MEDIAPIPE_ARCHIVE_PATH}" ]; then
  echo "Downloading MediaPipe..."
  wget "${MEDIAPIPE_URL}" -O "${MEDIAPIPE_ARCHIVE_PATH}"
fi

#
# Extract MediaPipe
#

echo "Extracting MediaPipe..."
rm -rf "${MEDIAPIPE_SOURCE_DIR}"
tar -zxf "${MEDIAPIPE_ARCHIVE_PATH}" --directory="${MEDIAPIPE_EXTRACT_DIR}"

#
# Patch MediaPipe
#

echo "Patching MediaPipe..."
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${MEDIAPIPE_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/mediapipe/0001-Enable-OpenCV-4.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${MEDIAPIPE_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/mediapipe/0002-Update-protobuf-to-version-3.21.12.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${MEDIAPIPE_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/mediapipe/0003-Enable-monolithic-build.patch"

#
# Build with Bazel
#

cd "${MEDIAPIPE_SOURCE_DIR}"

# If we’re on armhf, arm64/aarch64, etc. use clang; otherwise leave Bazel alone
BAZEL_CLANG_FLAGS=()
if [[ "${ARCH}" =~ ^(arm|armhf|arm64|aarch64)$ ]]; then
  BAZEL_CLANG_FLAGS+=(
    --action_env=CC=clang
    --action_env=CXX=clang++
  )
fi

# Build the monolithic library that we patched into MediaPipe
bazelisk build \
  "${BAZEL_CLANG_FLAGS[@]}" \
  --compilation_mode=opt \
  //mediapipe/framework:mediapipe_monolithic

#
# Install MediaPipe
#

# Clear old headers
rm -rf "${MEDIAPIPE_INSTALL_DIR}/include/absl"
rm -rf "${MEDIAPIPE_INSTALL_DIR}/include/mediapipe"

# Create the install layout
mkdir -p "${MEDIAPIPE_INSTALL_DIR}/include"
mkdir -p "${MEDIAPIPE_INSTALL_DIR}/lib"

# Copy headers from sources into the install include tree
(
  cd "${MEDIAPIPE_SOURCE_DIR}"
  find mediapipe -name '*.h' \
    -exec cp --parents '{}' "${MEDIAPIPE_INSTALL_DIR}/include/" \;
)

# Copy all generated .pb.h headers from the build directory into the install
# include tree
(
  cd "${MEDIAPIPE_BUILD_DIR}"
  find mediapipe -name '*.pb.h' \
    -exec cp --parents '{}' "${MEDIAPIPE_INSTALL_DIR}/include/" \;
)

# Copy Abseil headers into the install include tree
BAZEL_OUT_BASE=$(bazel info output_base)
ABSL_SOURCE_DIR="${BAZEL_OUT_BASE}/external/com_google_absl"
(
  cd "${ABSL_SOURCE_DIR}"
  find absl \( -name '*.h' -o -name '*.inc' \) \
    -exec cp --parents '{}' "${MEDIAPIPE_INSTALL_DIR}/include/" \;
)

# Install the monolithic libraries
echo "Installing libmediapipe_monolithic.so"
cp -f \
  "${MEDIAPIPE_BUILD_DIR}/mediapipe/framework/libmediapipe_monolithic.so" \
  "${MEDIAPIPE_INSTALL_DIR}/lib/"
echo "Installing libabsl_monolithic.so"
cp -f \
  "${MEDIAPIPE_BUILD_DIR}/mediapipe/framework/libabsl_monolithic.so" \
  "${MEDIAPIPE_INSTALL_DIR}/lib/"

echo "MediaPipe installed into ${MEDIAPIPE_INSTALL_DIR}"
