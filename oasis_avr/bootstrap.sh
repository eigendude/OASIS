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

#
# This is a script to bootstrap the Arduino toolchain needed to build the AVR
# package.
#
# Dependencies:
#
#   * bash
#   * curl
#   * git
#   * tar (except on macOS)
#   * unzip (only on macOS)
#

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

#
# Environment paths
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Location of the ROS workspace build directory
STACK_DIR="${SCRIPT_DIR}/.."
BUILD_DIR="${STACK_DIR}/ros-ws"

# Location of the Arduino CLI
ARDUINO_CLI_VERSION="1.3.1"
ARDUINO_CLI_DIR="${BUILD_DIR}/arduino-cli"

# Location of the Arduino CMake toolchain
TOOLCHAIN_DIR="${SCRIPT_DIR}/cmake/Arduino-CMake-Toolchain"

# Location of dependency source code
LIBRARY_DIR="${SCRIPT_DIR}/libraries"

# Location of dependency patches
PATCH_DIR="${LIBRARY_DIR}/patches"

#
# Import functions
#

source "${SCRIPT_DIR}/scripts/get_arduino_platform.sh"

#
# Helper functions
#

get_arduino_cli_platform() {
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

#
# Dependency configuration
#
# TODO: Move dependency mangement to CMake
#

ARDUINO_IDE_VERSION="1.8.19"
ARDUINO_IDE_PLATFORM="$(get_arduino_platform)"

if [[ "${OSTYPE}" != "darwin"* ]]; then
  ARDUINO_IDE_ARCHIVE="arduino-${ARDUINO_IDE_VERSION}-${ARDUINO_IDE_PLATFORM}.tar.xz"
else
  ARDUINO_IDE_ARCHIVE="arduino-${ARDUINO_IDE_VERSION}-${ARDUINO_IDE_PLATFORM}.zip"
fi

ARDUINO_IDE_URL="https://downloads.arduino.cc/${ARDUINO_IDE_ARCHIVE}"
ARDUINO_IDE_ARCHIVE_PATH="${SCRIPT_DIR}/${ARDUINO_IDE_ARCHIVE}"

# Location of the extracted Arduino IDE and toolchain
ARDUINO_IDE_DIR="${SCRIPT_DIR}/arduino-${ARDUINO_IDE_VERSION}"

# Location of the extracted Arduino CLI archive
ARDUINO_CLI_PLATFORM="$(get_arduino_cli_platform)"

if [ -z "${ARDUINO_CLI_PLATFORM}" ]; then
  echo "Unsupported platform for Arduino CLI: $(uname -s) $(uname -m)" >&2
  exit 1
fi

ARDUINO_CLI_ARCHIVE="arduino-cli_${ARDUINO_CLI_VERSION}_${ARDUINO_CLI_PLATFORM}.tar.gz"
ARDUINO_CLI_URL="https://downloads.arduino.cc/arduino-cli/${ARDUINO_CLI_ARCHIVE}"
ARDUINO_CLI_BIN="${ARDUINO_CLI_DIR}/arduino-cli"

# Location of the Adafruit BluefruitLE nRF15 library
ADAFRUIT_BLE_DIR="${LIBRARY_DIR}/Adafruit_BluefruitLE_nRF51"

# Location of the Adafruit Circuit Playground library included in the Arduino IDE
ADAFRUIT_CP_DIR="${ARDUINO_IDE_DIR}/libraries/Adafruit_Circuit_Playground"

# Location of the Firmata library included in the Arduino IDE
FIRMATA_DIR="${ARDUINO_IDE_DIR}/libraries/Firmata"

# Location of the FirmataExpress library
FIRMATA_EXPRESS_DIR="${LIBRARY_DIR}/FirmataExpress"

# Location of the TaskScheduler library
TASK_SCHEDULER_DIR="${LIBRARY_DIR}/TaskScheduler"

# Location of the i2cdevlib repo
I2CDEVLIB_DIR="${LIBRARY_DIR}/i2cdevlib"

#
# Toolchain setup
#

# Ensure working directories exist
mkdir -p "${BUILD_DIR}" "${ARDUINO_CLI_DIR}"

# Enter working directory
cd "${SCRIPT_DIR}"

echo "Updating git submodules..."
(
  cd "${SCRIPT_DIR}"
  git submodule update --init --recursive --force .
)

echo "Setting up Arduino CLI..."

INSTALLED_ARDUINO_CLI_VERSION=""
if [ -x "${ARDUINO_CLI_BIN}" ]; then
  INSTALLED_ARDUINO_CLI_VERSION_OUTPUT="$("${ARDUINO_CLI_BIN}" version || true)"
  INSTALLED_ARDUINO_CLI_VERSION="$(printf '%s\n' "${INSTALLED_ARDUINO_CLI_VERSION_OUTPUT}" | awk '/Version:/ {print $2; exit}')"
fi

if [ "${INSTALLED_ARDUINO_CLI_VERSION}" != "${ARDUINO_CLI_VERSION}" ]; then
  rm -f "${ARDUINO_CLI_BIN}"
fi

if [ ! -x "${ARDUINO_CLI_BIN}" ]; then
  echo "Downloading Arduino CLI ${ARDUINO_CLI_VERSION} (${ARDUINO_CLI_PLATFORM})..."
  curl --fail --location --show-error "${ARDUINO_CLI_URL}" --output "${ARDUINO_CLI_DIR}/${ARDUINO_CLI_ARCHIVE}"
  tar -xzf "${ARDUINO_CLI_DIR}/${ARDUINO_CLI_ARCHIVE}" --directory "${ARDUINO_CLI_DIR}"
  rm -f "${ARDUINO_CLI_DIR}/${ARDUINO_CLI_ARCHIVE}"
fi

if [ ! -x "${ARDUINO_CLI_BIN}" ]; then
  echo "Failed to set up Arduino CLI" >&2
  exit 1
fi

if [ ! -f "${HOME}/.arduino15/arduino-cli.yaml" ]; then
  echo "Initializing Arduino CLI configuration..."
  "${ARDUINO_CLI_BIN}" config init
fi

if ! "${ARDUINO_CLI_BIN}" core list | grep -q '^arduino:megaavr[[:space:]]'; then
  echo "Updating Arduino core index..."
  "${ARDUINO_CLI_BIN}" core update-index
  echo "Installing Arduino megaAVR core..."
  "${ARDUINO_CLI_BIN}" core install arduino:megaavr
else
  echo "Arduino megaAVR core already installed"
fi

if [ ! -d "${ARDUINO_IDE_DIR}" ]; then
  if [ -f "${ARDUINO_IDE_ARCHIVE_PATH}" ]; then
    echo "Using existing Arduino IDE archive ${ARDUINO_IDE_ARCHIVE}..."
  else
    echo "Downloading Arduino IDE..."
    curl --fail --location --show-error "${ARDUINO_IDE_URL}" --output "${ARDUINO_IDE_ARCHIVE_PATH}"
  fi

  if [[ "${OSTYPE}" != "darwin"* ]]; then
    tar -xJf "${ARDUINO_IDE_ARCHIVE_PATH}" --directory "${SCRIPT_DIR}"
  else
    # Not as graceful as piping to tar...
    # TODO: Add gnu-tar brew package to dependencies and use "gtar"
    unzip -o "${ARDUINO_IDE_ARCHIVE_PATH}"
    mv "${SCRIPT_DIR}/Arduino.app/Contents/Java" "${ARDUINO_IDE_DIR}"
    rm -rf "${SCRIPT_DIR}/Arduino.app"
  fi
fi

#
# Patch dependencies
#

# Patch Arduino-CMake-Toolchain
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${TOOLCHAIN_DIR}" \
  < "${PATCH_DIR}/Arduino-CMake-Toolchain/0001-Add-variable-ARDUINO_BOARD_RAM_SIZE.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${TOOLCHAIN_DIR}" \
  < "${PATCH_DIR}/Arduino-CMake-Toolchain/0002-Fix-Arduino-toolchain-compatibility-with-CMake-4.0.patch"

# Patch Adafruit BluefruitLE_nRF51 library
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${ADAFRUIT_BLE_DIR}" \
  < "${PATCH_DIR}/Adafruit_BluefruitLE_nRF51/0001-Fix-library-not-located-by-Arduino-CMake-Toolchain.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${ADAFRUIT_BLE_DIR}" \
  < "${PATCH_DIR}/Adafruit_BluefruitLE_nRF51/0001-Add-function-to-get-Bluefruit-info.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${ADAFRUIT_BLE_DIR}" \
  < "${PATCH_DIR}/Adafruit_BluefruitLE_nRF51/0001-Fix-hang.patch"

# Patch FirmataExpress
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${FIRMATA_EXPRESS_DIR}" \
  < "${PATCH_DIR}/FirmataExpress/0001-Remove-deprecated-pin-modes-due-to-define-conflicts.patch"

# Because we use FirmataExpress instead of Firmata, remove the Firmata library
# to avoid confusing IDEs
if [ -d "${FIRMATA_DIR}" ]; then
  echo "Removing ${FIRMATA_DIR}"
  rm -rf "${FIRMATA_DIR}"
fi

# Similarly, Adafruit Circuit Playground bundles a modified Firmata
if [ -d "${ADAFRUIT_CP_DIR}" ]; then
  echo "Removing ${ADAFRUIT_CP_DIR}"
  rm -rf "${ADAFRUIT_CP_DIR}"
fi

# TaskSheduler includes and Arduino.h with its tests
if [ -f "${TASK_SCHEDULER_DIR}/tests/Arduino.h" ]; then
  echo "Removing ${TASK_SCHEDULER_DIR}/tests/Arduino.h"
  rm -f "${TASK_SCHEDULER_DIR}/tests/Arduino.h"
fi

# Remove unused platforms that can confuse IDEs
for dir in \
  "${I2CDEVLIB_DIR}/BeagleBoneBlack/" \
  "${I2CDEVLIB_DIR}/dsPIC30F" \
  "${I2CDEVLIB_DIR}/EFM32" \
  "${I2CDEVLIB_DIR}/ESP32_ESP-IDF" \
  "${I2CDEVLIB_DIR}/Jennic" \
  "${I2CDEVLIB_DIR}/LinuxI2CDev" \
  "${I2CDEVLIB_DIR}/MSP430" \
  "${I2CDEVLIB_DIR}/nRF51" \
  "${I2CDEVLIB_DIR}/PIC18" \
  "${I2CDEVLIB_DIR}/RaspberryPi_bcm2835" \
  "${I2CDEVLIB_DIR}/RP2040" \
  "${I2CDEVLIB_DIR}/STM32" \
  "${I2CDEVLIB_DIR}/STM32HAL" \
; do
  if [ -d "${dir}" ]; then
    echo "Removing ${dir}"
    rm -rf "${dir}"
  fi
done
