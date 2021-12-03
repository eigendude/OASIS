################################################################################
#
#  Copyright (C) 2016-2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import subprocess

from oasis_drivers.network.interface import Interface
from oasis_drivers.network.interface import InterfaceType


# Linux parameters
IP_BINARY_PATH: str = "/bin/ip"


class InterfaceBridge(Interface):
    def __init__(self, name, logger) -> None:
        super().__init__(name)

        self._logger = logger
        self._log_cap_net_admin = True

    def type(self) -> InterfaceType:
        return InterfaceType.BRIDGE

    def initialize(self) -> bool:
        # Check if ip has CAP_NET_ADMIN capability
        proc = subprocess.Popen(["getcap", IP_BINARY_PATH], stdout=subprocess.PIPE)
        stdout, stderr = proc.communicate()

        has_net_admin_capability = False

        if b"cap_net_admin+ep" in stdout:
            has_net_admin_capability = True
        elif self._log_cap_net_admin:
            self._logger.error("")
            self._logger.error(
                "*******************************************************"
            )
            self._logger.error("Error: This process requires CAT_NET_ADMIN on ip:")
            self._logger.error("")
            self._logger.error(f"sudo setcap cap_net_admin+ep {IP_BINARY_PATH}")
            self._logger.error(
                "*******************************************************"
            )
            self._logger.error("")
            self._log_cap_net_admin = False

        return has_net_admin_capability

    def add_interface(self, interface: Interface) -> None:
        self._logger.info(f"Adding [{interface.name()}] to bridge [{self.name()}]")
        subprocess.Popen(
            [IP_BINARY_PATH, "link", "set", interface.name(), "master", self.name()]
        )

    def remove_interface(self, interface: Interface) -> None:
        self._logger.info(f"Removing [{interface.name()}] from bridge [{self.name()}]")
        subprocess.Popen([IP_BINARY_PATH, "link", "set", interface.name(), "nomaster"])
