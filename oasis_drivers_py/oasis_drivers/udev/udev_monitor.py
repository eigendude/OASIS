################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import pyudev


class UdevMonitor(object):
    """
    Synchronously monitor device events

    A pyudev.Monitor objects connects to the udev daemon and listens for
    changes to the device list.
    """

    def __init__(self, udev_context: pyudev.Context) -> None:
        """
        Create the monitor

        :param udev_context: The pyudev context
        """
        self._input_monitor: pyudev.Monitor = pyudev.Monitor.from_netlink(udev_context)

    def get_resource(self) -> pyudev.Monitor:
        """
        Access the monitor
        """
        return self._input_monitor
