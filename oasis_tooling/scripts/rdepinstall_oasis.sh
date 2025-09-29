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
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

#
# Load OASIS dependency environment
#

set +o nounset
source "${OASIS_DEPENDS_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

#
# Install OASIS runtime dependencies
#

# Install display controllers
if [[ ${PLATFORM_ARCH} == i*86 ]] || [[ ${PLATFORM_ARCH} == x86_64 ]]; then
  sudo apt install -y --no-install-recommends \
    cec-utils \
    ddcutil \
    vbetool \
; fi

# Install Python dependencies
sudo apt install -y --no-install-recommends \
  python3-psutil \
  python3-serial \

sudo python3 -m pip install \
  --upgrade \
  --ignore-installed \
  --break-system-packages \
  mediapipe \
  telemetrix-aio \

# Needed for resolving hostnames for WoL
sudo apt install -y --no-install-recommends \
  avahi-daemon \
  avahi-utils \
