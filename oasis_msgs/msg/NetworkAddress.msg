################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Telemetry about a network address
#

# The adress family, e.g. IPv4, IPV6, etc.
string family

# The network address
string address

# The netmask (used to mask the address for logical subnets)
string netmask
