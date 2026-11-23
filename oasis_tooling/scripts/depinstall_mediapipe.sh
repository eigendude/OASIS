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

# Get the absolute path to this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

#
# Version parameters
#

BAZELISK_VERSION="1.30.0"

#
# Install dependencies
#

if [[ "${OSTYPE}" == "darwin"* ]]; then
  brew install glog protobuf
else
  # Packages to install via apt
  APT_PACKAGES=(
    curl
    git

    # EGL/GLES for MediaPipe GPU/OpenGL paths
    libegl-dev
    libgles-dev

    # OpenCV transitive deps pulled by the broad MediaPipe pose monolith
    libavcodec-dev
    libavdevice-dev
    libavformat-dev
    libavif-dev
    libavutil-dev
    libceres-dev
    libgstreamer-plugins-base1.0-dev
    libgstreamer1.0-dev
    libhdf5-dev
    libleptonica-dev
    libopenblas-dev
    libopenexr-dev
    libswscale-dev
    libtesseract-dev

    # Protobuf tools/runtime used at the OASIS/OpenCV boundary
    protobuf-compiler
    libprotobuf-dev

    # Python tooling used by MediaPipe/Bazel
    python3-dev
    python3-venv
    python3-wheel

    # System glog
    libgoogle-glog-dev
  )

  sudo apt update
  sudo apt install -y --no-install-recommends "${APT_PACKAGES[@]}"

  # Remove the PyPI MediaPipe wheel before installing the source build
  MEDIAPIPE_PIP_PACKAGES=(
    mediapipe
  )

  sudo python3 -m pip uninstall -y \
    --break-system-packages \
    "${MEDIAPIPE_PIP_PACKAGES[@]}"
fi

# Use the same Python environment as OpenCV
source "${SCRIPT_DIR}/env_mediapipe.sh"

# Install or validate the shared NumPy installation
NUMPY_PYTHON_BIN="python3" \
  "${SCRIPT_DIR}/install_numpy.sh"

# Install the pinned native launcher for the build host
if [[ "${OSTYPE}" == "darwin"* ]]; then
  python3 -m pip install setuptools
  python3 -m pip install --upgrade wheel

  # The universal release contains both arm64 and x86_64 executables
  BAZELISK_ASSET="bazelisk-darwin"
  BAZELISK_SHA256="7d0447135f7bbcee2905654544f94c25eeec4f70ba61fb898238f54e9b3b26c6"
  SHA256_COMMAND=(shasum -a 256)
else
  BAZELISK_ASSET="bazelisk-linux-${ARCH}"
  SHA256_COMMAND=(sha256sum)
  case "${ARCH}" in
    amd64) BAZELISK_SHA256="a248ae2b2b62b5baebb748f6fba7203bff1a27e5d01b55399a8406744cb539e8" ;;
    arm64) BAZELISK_SHA256="b2bcc040c213b44586b121bda6d983c35df121b917c3a7a66fb16f2fa0df1a4f" ;;
    *) echo "No native Bazelisk release for Linux architecture '${ARCH}'" >&2; exit 1 ;;
  esac
fi

mkdir -p "${MEDIAPIPE_BIN_DIR}"
BAZELISK_BIN="${MEDIAPIPE_BIN_DIR}/bazelisk"
if [[ ! -x "${BAZELISK_BIN}" ]] ||
  ! "${SHA256_COMMAND[@]}" --check --status <<<"${BAZELISK_SHA256}  ${BAZELISK_BIN}"; then
  echo "Installing ${BAZELISK_ASSET} ${BAZELISK_VERSION}..."
  BAZELISK_TEMP="$(mktemp "${MEDIAPIPE_BIN_DIR}/.bazelisk.XXXXXXXX")"
  trap 'rm -f -- "${BAZELISK_TEMP}"' EXIT
  curl -fsSL --retry 3 \
    "https://github.com/bazelbuild/bazelisk/releases/download/v${BAZELISK_VERSION}/${BAZELISK_ASSET}" \
    -o "${BAZELISK_TEMP}"
  "${SHA256_COMMAND[@]}" --check <<<"${BAZELISK_SHA256}  ${BAZELISK_TEMP}"
  chmod 755 "${BAZELISK_TEMP}"
  mv -f -- "${BAZELISK_TEMP}" "${BAZELISK_BIN}"
fi

# MediaPipe's Python build invokes the launcher as bazel
ln -sfn bazelisk "${MEDIAPIPE_BIN_DIR}/bazel"
