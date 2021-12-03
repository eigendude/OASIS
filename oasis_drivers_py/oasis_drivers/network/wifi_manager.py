#!/usr/bin/env python3
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

from oasis_drivers.network.wifi_interface import WiFiInterface


# Linux parameters
IW_BINARY_PATH = "/sbin/iw"


class WiFiManager:
    """
    Manages WiFi scanning and connections.

    Dependencies:

      * iw - Needs CAP_NET_ADMIN capability

    """

    def __init__(self, logger):
        self._logger = logger
        self._scan_interfaces = []

    def initialize(self):
        # Check if iw has CAP_NET_ADMIN capability
        proc = subprocess.Popen(["getcap", IW_BINARY_PATH], stdout=subprocess.PIPE)
        stdout, stderr = proc.communicate()

        has_net_admin_capability = False

        if b"cap_net_admin+ep" in stdout:
            has_net_admin_capability = True
        else:
            self._logger.error("")
            self._logger.error(
                "*******************************************************"
            )
            self._logger.error("Error: This process requires CAT_NET_ADMIN on iw:")
            self._logger.error("")
            self._logger.error(f"sudo setcap cap_net_admin+ep {IW_BINARY_PATH}")
            self._logger.error(
                "*******************************************************"
            )
            self._logger.error("")

        return has_net_admin_capability

    def check_is_wireless(self, device):
        proc = subprocess.Popen(
            [IW_BINARY_PATH, "dev", device, "scan", "dump"], stdout=subprocess.PIPE
        )
        proc.communicate()
        return proc.returncode == 0

    def start_scan(self, interface):
        if interface not in self._scan_interfaces:
            self._scan_interfaces.append(interface)

    def end_scan(self, interface):
        if interface in self._scan_interfaces:
            self._scan_interfaces.remove(interface)

    def get_interfaces(self):
        return [WiFiInterface(name) for name in self._scan_interfaces]
