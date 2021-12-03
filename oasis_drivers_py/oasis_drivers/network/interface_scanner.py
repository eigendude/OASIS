################################################################################
#
#  Copyright (C) 2016-2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from typing import Optional

import netifaces

from oasis_drivers.network.interface import Interface
from oasis_drivers.network.interface_ethernet import InterfaceEthernet
from oasis_drivers.network.interface_tap import InterfaceTap
from oasis_drivers.network.interface_wifi import InterfaceWiFi


class InterfaceScanner:
    def __init__(self, callback, logger):
        self._callback = callback
        self._logger = logger
        self._interfaces = {}

    def run_scan(self) -> None:
        scan_results = netifaces.interfaces()

        # Check for  interface changes
        removed_interfaces = [
            name for name in self._interfaces if name not in scan_results
        ]
        added_interfaces = [
            name for name in scan_results if name not in self._interfaces
        ]

        # Handle removed interfaces
        for name in removed_interfaces:
            interface = self._interfaces[name]
            self._callback.interface_removed(interface)
            self._interfaces.pop(name)
            interface.deinitialize()

        # Handle added interfaces
        for name in added_interfaces:
            iface: Optional[Interface] = None

            # Skip loopback interface
            if name == "lo":
                continue

            # Handle tap device
            elif InterfaceTap.is_tap(name):
                iface = InterfaceTap(name)

            # TODO: Handle bridges

            # Handle WiFI
            elif InterfaceWiFi.check_is_wireless(name):
                iface = InterfaceWiFi(name, self._logger)
            else:
                iface = InterfaceEthernet(name)

            if iface and iface.initialize():
                self._interfaces[iface.name()] = iface
                self._callback.interface_added(iface)
