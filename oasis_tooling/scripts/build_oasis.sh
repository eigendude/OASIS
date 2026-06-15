#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

usage() {
  cat <<'USAGE'
Usage: build_oasis.sh [OPTIONS]

Options:
  --mcu-profile PROFILE   Build oasis_mcu using the named MCU profile.
  --skip-mcu               Skip building the oasis_mcu package. Use this when
                           pre-built libraries have been restored from cache.
  --skip-drivers-cpp       Skip building the oasis_drivers_cpp package. Use
                           this when pre-built libraries have been restored
                           from cache.
  --skip-messages          Skip building the oasis_msgs package. Use this when
                           pre-built messages have been restored from cache.
  --skip-perception-cpp    Skip building the oasis_perception_cpp package. Use
                           this when pre-built libraries have been restored
                           from cache.
  -h, --help               Display this help and exit.
USAGE
}

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

SKIP_MCU=false
SKIP_DRIVERS_CPP=false
SKIP_MESSAGES=false
SKIP_PERCEPTION_CPP=false
MCU_PROFILE=

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mcu-profile)
      if [[ $# -lt 2 ]]; then
        echo "Error: --mcu-profile requires a profile name" >&2
        usage >&2
        exit 2
      fi
      MCU_PROFILE="$2"
      shift 2
      ;;
    --skip-mcu)
      SKIP_MCU=true
      shift
      ;;
    --skip-drivers-cpp)
      SKIP_DRIVERS_CPP=true
      shift
      ;;
    --skip-messages)
      SKIP_MESSAGES=true
      shift
      ;;
    --skip-perception-cpp)
      SKIP_PERCEPTION_CPP=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Error: Unknown option '$1'" >&2
      usage >&2
      exit 2
      ;;
  esac
done

#
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import CMake paths and config
source "${SCRIPT_DIR}/env_cmake.sh"

# Import OpenCV paths and config
source "${SCRIPT_DIR}/env_cv.sh"

# Import MediaPipe paths and config
source "${SCRIPT_DIR}/env_mediapipe.sh"

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

#
# Load OASIS dependency environment
#

set +o nounset
source "${OASIS_DEPENDS_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

#
# Build configuration
#

# Default values
MAKE_FLAGS=
COLCON_FLAGS="--merge-install"

# Add ccache support
COLCON_FLAGS+=" \
  --cmake-args \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH} \
"

if [[ -n "${MCU_PROFILE}" ]]; then
  COLCON_FLAGS+=" -DOASIS_MCU_PROFILE=${MCU_PROFILE}"
fi

# macOS flags
if [[ "${OSTYPE}" == "darwin"* ]]; then
  # macOS packges don't all build, so do a best effort to build as much as
  # possible
  COLCON_FLAGS+=" --continue-on-error"
fi

PACKAGES_SKIP=()

if [[ "${SKIP_MCU}" == true ]]; then
  PACKAGES_SKIP+=("oasis_mcu")
fi

if [[ "${SKIP_DRIVERS_CPP}" == true ]]; then
  PACKAGES_SKIP+=("oasis_drivers_cpp")
fi

if [[ "${SKIP_MESSAGES}" == true ]]; then
  PACKAGES_SKIP+=("oasis_msgs")
fi

if [[ "${SKIP_PERCEPTION_CPP}" == true ]]; then
  PACKAGES_SKIP+=("oasis_perception_cpp")
fi

if [[ ${#PACKAGES_SKIP[@]} -gt 0 ]]; then
  COLCON_FLAGS+=" --packages-skip ${PACKAGES_SKIP[*]}"
fi

# Uncomment these to force building in serial
#MAKE_FLAGS+=" -j1 -l1"
#COLCON_FLAGS+=" --executor sequential --event-handlers console_direct+"

#
# Directory setup
#

# Ensure directories exist
mkdir -p "${OASIS_DIRECTORY}"

# Symlink the source directory
if [ ! -L "${OASIS_SOURCE_DIRECTORY}" ]; then
  rm -rf "${OASIS_SOURCE_DIRECTORY}"
  ln -s "${STACK_DIRECTORY}" "${OASIS_SOURCE_DIRECTORY}"
fi

#
# Build OASIS with colcon
#

(
  cd "${OASIS_DIRECTORY}"

  MAKE_FLAGS="${MAKE_FLAGS}" \
    colcon build \
      ${COLCON_FLAGS}
)

#
# Stamp install-time values into installed systemd service templates
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  shopt -s nullglob

  for SYSTEMD_SERVICE in "${OASIS_INSTALL_DIRECTORY}/share"/*/systemd/*.service; do
    echo "Stamping $(basename -- "${SYSTEMD_SERVICE}")"

    sed -i \
      -e "s|@ROS2_DISTRO@|${ROS2_DISTRO}|g" \
      -e "s|^User=.*|User=$(id -un)|" \
      "${SYSTEMD_SERVICE}"
  done

  shopt -u nullglob
fi
