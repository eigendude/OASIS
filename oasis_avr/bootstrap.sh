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

#
# This is a script to bootstrap the Arduino toolchain needed to build the AVR
# package.
#
# Dependencies:
#
#   * bash
#   * curl
#   * git
#   * tar
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

# Location of dependency source code
LIBRARY_DIR="${SCRIPT_DIR}/libraries"

# Location of dependency patches
PATCH_DIR="${LIBRARY_DIR}/patches"

#
# Import functions
#

source "${SCRIPT_DIR}/scripts/get_arduino_platform.sh"

#
# Dependency configuration
#
# TODO: Move dependency mangement to CMake
#

ARDUINO_IDE_VERSION="1.8.18"
ARDUINO_IDE_PLATFORM="$(get_arduino_platform)"
ARDUINO_IDE_URL="https://downloads.arduino.cc/arduino-${ARDUINO_IDE_VERSION}-${ARDUINO_IDE_PLATFORM}.tar.xz"

# Location of the extracted Arduino IDE and toolchain
ARDUINO_IDE_DIR="${SCRIPT_DIR}/arduino-${ARDUINO_IDE_VERSION}"

# Location of the Firmata library included in the Arduino IDE
FIRMATA_DIR="${ARDUINO_IDE_DIR}/libraries/Firmata"

# Location of the Adafruit Circuit Playground library included in the Arduino IDE
ADAFRUIT_CP_DIR="${ARDUINO_IDE_DIR}/libraries/Adafruit_Circuit_Playground"

#
# Toolchain setup
#

# Enter working directory
cd "${SCRIPT_DIR}"

echo "Updating git submodules..."
git submodule update --init --recursive --force -- "${SCRIPT_DIR}"

if [ ! -d "${ARDUINO_IDE_DIR}" ]; then
  echo "Downloading Arduino IDE..."
  curl "${ARDUINO_IDE_URL}" | tar -xJ --directory "${SCRIPT_DIR}"
fi

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
