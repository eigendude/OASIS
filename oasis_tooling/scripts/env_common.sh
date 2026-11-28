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

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

#
# Environment configuration
#

PLATFORM_ARCH="$(uname -p)"

if [[ "${OSTYPE}" != "darwin"* ]]; then
  ARCH="$(dpkg --print-architecture)"
  CODENAME="$(source "/etc/os-release" && echo "${UBUNTU_CODENAME}")"
else
  ARCH=
  CODENAME=
fi

#
# Directory and path definitions
#

# Get the absolute path to this script
SCRIPT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

# Directory of the ROS 2 package
PACKAGE_DIRECTORY="${SCRIPT_DIR}/.."

# Directory of package configuration
CONFIG_DIRECTORY="${PACKAGE_DIRECTORY}/config"

# Directory of the ROS 2 stack (OASIS repo)
STACK_DIRECTORY="${PACKAGE_DIRECTORY}/.."

# Subdirectory for ROS build files
BUILD_DIRECTORY="${STACK_DIRECTORY}/ros-ws"

#
# Environment setup
#

# Import RPi environment variables in case we're running on an RPi
source "${SCRIPT_DIR}/env_rpi.sh"

#
# Directory setup
#

# Create directories
mkdir -p "${BUILD_DIRECTORY}"

# Exclude build directory from rosdep installs
touch "${BUILD_DIRECTORY}/CATKIN_IGNORE"

# Exclude build directory from Colcon builds
touch "${BUILD_DIRECTORY}/COLCON_IGNORE"

#
# Hardware config
#

# The physical memory in KiB
if [[ "${OSTYPE}" == "darwin"* ]]; then
  PHYSICAL_MEMORY_KB=$(sysctl -n hw.memsize | awk '{printf "%d", $1/1024}')
else
  PHYSICAL_MEMORY_KB=$(grep MemTotal /proc/meminfo | awk '{print $2}')
fi

# The physical memory in GiB
# Use awk instead of bc so we don’t depend on the bc package
# 1024 × 1024 = 1 048 576 KiB per GiB
PHYSICAL_MEMORY_GB=$(awk -v kb="${PHYSICAL_MEMORY_KB}" 'BEGIN {printf "%.4f", kb/1048576}')

# Prepare the pinned checkout and apply the supplied patches in order.
# Patch paths must be absolute. OASIS_SOURCE_OUTPUTS lists Git exclusions for
# generated files. OASIS_SOURCE_CONFIGURE can supply additional source edits,
# with OASIS_SOURCE_KEY identifying the inputs to those edits.
prepare_git_source() (
  local source_dir="$1" repository="$2" revision="$3"
  shift 3
  local refspec="${revision}" receipt pin key tree extras
  local previous='' previous_repository='' patch_file

  if [[ ! -e "${source_dir}/.git" ]]; then
    git init --quiet "${source_dir}" || return 1
  fi
  cd "${source_dir}" || return 1
  receipt="$(git rev-parse --git-path oasis-prepared)" || return 1
  if [[ -f "${receipt}" ]]; then
    read -r previous tree previous_repository < "${receipt}" || previous=''
  fi
  if [[ -n "${previous_repository}" && "${repository}" != "${previous_repository}" ]] ||
     ! git rev-parse --verify --quiet "${revision}^{commit}" > /dev/null; then
    if [[ "${revision}" == refs/tags/* ]]; then
      refspec="+${revision}:${revision}"
    fi
    git fetch --depth=1 --no-tags "${repository}" "${refspec}" || return 1
  fi

  pin="$(git rev-parse "${revision}^{commit}")" || return 1
  key="$(
    {
      printf '%s\0' "${pin}" "${OASIS_SOURCE_KEY:-}"
      git hash-object -- "$@"
    } | git hash-object --stdin
  )" || return 1
  if [[ "${previous}" == "${key}" && "${previous_repository}" == "${repository}" &&
        "$(git rev-parse HEAD)" == "${pin}" ]] && git diff --quiet "${tree}" --; then
    extras="$(git ls-files --others --directory ${OASIS_SOURCE_OUTPUTS[@]+"${OASIS_SOURCE_OUTPUTS[@]}"})" || return 1
    if [[ -z "${extras}" ]]; then
      echo "Reusing ${source_dir} at ${revision}"
      return
    fi
  fi

  rm -f "${receipt}" || return 1
  git checkout --quiet --force --detach "${pin}" || return 1
  git clean -qffdx || return 1
  for patch_file in "$@"; do
    # Kodi may already include an OASIS patch in its pinned upstream commit
    if patch -p1 --batch --dry-run --forward < "${patch_file}" > /dev/null 2>&1; then
      patch -p1 --batch --forward --no-backup-if-mismatch < "${patch_file}" || return 1
    else
      patch -p1 --force --dry-run --reverse < "${patch_file}" || return 1
    fi
  done
  "${OASIS_SOURCE_CONFIGURE:-:}"
  # Include new source files introduced by patches in the expected tree
  git add -f -A . || return 1
  tree="$(git write-tree)" || return 1
  printf '%s %s %s\n' "${key}" "${tree}" "${repository}" > "${receipt}"
)
