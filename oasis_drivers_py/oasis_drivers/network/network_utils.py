################################################################################
#
#  Copyright (C) 2021-2026 Garrett Brown
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
    def get_network_address_family(socket_kind: int | socket.AddressFamily) -> str:
        address_families: dict[int, str] = {
            socket.AF_INET: "IPv4",
            socket.AF_INET6: "IPv6",
        }
        af_packet: int | None = getattr(socket, "AF_PACKET", None)
        if af_packet is not None:
            address_families[af_packet] = "MAC"

        family = address_families.get(int(socket_kind))
        if family is not None:
            return family

        # Stringify constant for discovery in logs
        return str(socket_kind)
