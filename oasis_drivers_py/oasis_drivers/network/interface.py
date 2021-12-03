################################################################################
#
#  Copyright (C) 2016-2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import enum
from typing import Optional

import netifaces


class InterfaceType(enum.Enum):
    UNKNOWN = 0
    WIFI = 1
    ETHERNET = 2
    BRIDGE = 3
    TAP = 4


class Interface(object):
    def __init__(self, name: str):
        self._name: str = name

    def name(self) -> str:
        return self._name

    def type(self) -> InterfaceType:
        return InterfaceType.UNKNOWN

    def initialize(self) -> bool:
        return True

    def deinitialize(self) -> None:
        pass

    def get_address(self) -> Optional[str]:
        addresses = netifaces.ifaddresses(self._name)

        if netifaces.AF_INET in addresses:
            for address in addresses[netifaces.AF_INET]:
                # Skip invalid addresses
                if address["addr"] == "0.0.0.0":
                    continue

                # Skip link-local addresses
                if address["addr"].startswith("169.254"):
                    continue

                return address["addr"]

        return None

    def get_gateway(self) -> Optional[str]:
        gateways = netifaces.gateways()

        if netifaces.AF_INET in gateways:
            for gateway in gateways[netifaces.AF_INET]:
                interface = gateway[1]
                if self._name == interface:
                    ip_address: str = gateway[0]
                    return ip_address

        return None
