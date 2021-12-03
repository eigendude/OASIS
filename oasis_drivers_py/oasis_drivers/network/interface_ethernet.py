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


class InterfaceEthernet(Interface):
    def __init__(self, name):
        super().__init__(name)

    def type(self):
        return InterfaceType.ETHERNET
