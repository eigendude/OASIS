#!/bin/bash
################################################################################
#
#  Copyright (C) 2023-2025 Garrett Brown
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

################################################################################
# Environment configuration
################################################################################

# Location of the swap storage
SWAP_FILE="/swapfile"

# Size of swap storage (get a big SD card)
SWAP_SIZE="16G"

################################################################################
# Directory and path definitions
################################################################################

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

################################################################################
# Environment setup
################################################################################

# Import RPi environment variables
source "${SCRIPT_DIR}/env_rpi.sh"

################################################################################
# Create swap storage
################################################################################

if ! grep -q "swap" "/etc/fstab"; then
  if ! [ -f "${SWAP_FILE}" ]; then
    echo "Creating ${SWAP_FILE}"
    sudo fallocate --length "${SWAP_SIZE}" "${SWAP_FILE}"
    sudo chmod 600 "${SWAP_FILE}"
    sudo mkswap "${SWAP_FILE}"
    sudo swapon "${SWAP_FILE}"
  fi

  echo "Adding swapfile to /etc/fstab"
  echo "${SWAP_FILE}    none    swap    sw    0   0" | sudo tee --append /etc/fstab
else
  echo "Swapfile already exists. Contents of /etc/fstab:"
  echo -n "  "
  grep "swap" "/etc/fstab"

  # Get the path to the swapfile
  EXISTING_SWAP_FILE=$(cat "/etc/fstab" | grep "swap" | awk '{print $1}')

  # Get the size of the swapfile
  SWAP_SIZE=$(ls -lh "${EXISTING_SWAP_FILE}" | awk '{print $5}')

  # Print the size of the swapfile
  echo "Size of swapfile:"
  echo -n "  "
  echo "${SWAP_SIZE}"
fi

################################################################################
# Disable interactive mode for apt
################################################################################

sudo bash -c "echo 'Defaults    env_keep += \"NEEDRESTART_MODE\"' >/etc/sudoers.d/oasis-users"

echo
echo "Add the following line to your ~/.bashrc file:"
echo 'export NEEDRESTART_MODE="a"'
echo

################################################################################
# Disable the wait-online service to prevent the system from waiting on a
# network connection and prevent the service from starting if requested by
# another service
################################################################################

sudo systemctl disable systemd-networkd-wait-online.service
sudo systemctl mask systemd-networkd-wait-online.service
