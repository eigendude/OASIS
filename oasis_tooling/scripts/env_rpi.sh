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

#
# Disable interactive mode for apt
#
# In a normal install, this variable as no effect, because sudo doesn't keep
# environment variables by default. In order to pass this variable to sudo,
# add the following to your sudoers file (i.e. sudo visudo):
#
#   Defaults        env_keep += "NEEDRESTART_MODE"
#

export NEEDRESTART_MODE="a"
