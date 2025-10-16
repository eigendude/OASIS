#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
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
# This function detects the system parameters and returns an appropriate Arduino
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

function get_arduino_ide_platform() {
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

#
# This function detects the system parameters and returns an appropriate
# Arduino CLI platform. If it fails to detect a platform, it returns an
# empty string.
#

function get_arduino_cli_platform() {
  local kernel_name
  kernel_name="$(uname -s)"

  case "${kernel_name}" in
    Linux)
      case "$(uname -m)" in
        x86_64)
          echo "Linux_64bit"
          ;;
        i386|i686)
          echo "Linux_32bit"
          ;;
        armv6l)
          echo "Linux_ARMv6"
          ;;
        armv7l|armv7*)
          echo "Linux_ARMv7"
          ;;
        aarch64|arm64)
          echo "Linux_ARM64"
          ;;
        *)
          echo ""
          ;;
      esac
      ;;
    Darwin)
      echo "macOS_64bit"
      ;;
    MINGW*|MSYS*|CYGWIN*)
      echo "Windows_64bit"
      ;;
    *)
      echo ""
      ;;
  esac
}
