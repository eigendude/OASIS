################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Telemetry about a network interface
#

# Name used for the public network interface
string PUBLIC_NETWORK_INTERFACE=wan

# The interface name, or PUBLIC_NETWORK_INTERFACE for the public interface
string name

# The provider of the network interface (e.g. NIC name or ISP), if known
string provider

# The number of bytes sent through this interface since boot
uint64 bytes_sent

# The number of bytes received through this interface since boot
uint64 bytes_received

# The addresses assigned to this interface
NetworkAddress[] addresses
