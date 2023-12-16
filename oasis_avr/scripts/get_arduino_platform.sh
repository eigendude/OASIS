#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

#
# This script detects the system parameters and returns an appropriate Arduino
# IDE platform. If it fails to detect a platform, it exits non-zero.
#
# The available Arduino IDE platforms are:
#
#   linux32
#   linux64
#   linuxarm
#   linuxaarch64
#   macosx
#   windows
#
# Dependencies:
#
#   * bash
#   * uname (on Linux/FreeBSD)
#

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

function get_arduino_platform() {
  ARDUINO_PLATFORM=""

  if [[ "${OSTYPE}" == "linux-gnu"* ]] || [[ "${OSTYPE}" == "freebsd"* ]]; then
    MACHINE_TYPE=$(uname -m)
    case "${MACHINE_TYPE}" in
      x86 | i?86)
        ARDUINO_PLATFORM="linux32"
        ;;
      amd64 | x86_64)
        ARDUINO_PLATFORM="linux64"
        ;;
      arm | armv7l)
        ARDUINO_PLATFORM="linuxarm"
        ;;
      arm64 | aarch64)
        ARDUINO_PLATFORM="linuxaarch64"
        ;;
      *)
        >&2 echo "Unknown machine type: ${MACHINE_TYPE}"
        exit 1
        ;;
    esac
  elif [[ "${OSTYPE}" == "darwin"* ]]; then
    ARDUINO_PLATFORM="macosx"
  elif [[ "${OSTYPE}" == "cygwin" ]]; then
    # POSIX compatibility layer and Linux environment emulation for Windows
    ARDUINO_PLATFORM="windows"
  elif [[ "${OSTYPE}" == "msys" ]]; then
    # Lightweight shell and GNU utilities compiled for Windows (part of MinGW)
    ARDUINO_PLATFORM="windows"
  fi

  if [ -z "${ARDUINO_PLATFORM}" ]; then
    >&2 echo "Unknown OS: ${OSTYPE}"
    exit 2
  fi

  echo "${ARDUINO_PLATFORM}"
}
