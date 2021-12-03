################################################################################
#
#  Copyright (C) 2016-2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from oasis_drivers.network.interface import Interface
from oasis_drivers.network.interface import InterfaceType


# Prefix for tap devices, e.g. 'tap' for tap0
TAP_NAME_PREFIX: str = "tap"


class InterfaceTap(Interface):
    def __init__(self, name) -> None:
        super().__init__(name)

    def type(self) -> InterfaceType:
        return InterfaceType.TAP

    @staticmethod
    def is_tap(name: str) -> bool:
        return name.startswith(TAP_NAME_PREFIX)
