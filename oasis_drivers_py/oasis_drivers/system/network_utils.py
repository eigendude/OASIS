#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import socket


################################################################################
# Network utility functions
################################################################################


class NetworkUtils:
    @staticmethod
    def get_network_address_family(socket_kind: socket.AddressFamily) -> str:
        try:
            return {
                socket.AddressFamily.AF_INET: "IPv4",
                socket.AddressFamily.AF_INET6: "IPv6",
                socket.AddressFamily.AF_PACKET: "MAC",
            }[socket_kind]
        except KeyError:
            pass

        # Stringify constant for discovery in logs
        return str(socket_kind)
