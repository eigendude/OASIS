################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from typing import TYPE_CHECKING

import pyudev


if TYPE_CHECKING:
    from oasis_drivers.udev.udev_monitor import UdevMonitor
    from oasis_drivers.udev.udev_process import UdevProcess


class UdevMonitorObserver(object):
    """
    An asynchronous observer for device events.

    pyudev.MonitorObserver subclasses Thread class to asynchronously observe
    a pyudev.Monitor in a background thread.
    """

    def __init__(
        self, udev_process: "UdevProcess", udev_monitor: "UdevMonitor"
    ) -> None:
        """
        Create a new udev monitor observer.
        """
        # Construction parameters
        self._udev_process = udev_process
        self._udev_monitor = udev_monitor

        # Initialize udev
        self._observer = pyudev.MonitorObserver(
            self._udev_monitor.get_resource(), self._event_handler
        )

    def _event_handler(self, action: str, device: pyudev.Device) -> None:
        """
        Handle a dev event. Events arrive asynchronously on every event
        emitted by the underlying monitor.

        :param action: The udev action
        :param device: The udev device
        """
        self._udev_process.handle_event(action, device)

    def start(self) -> None:
        self._observer.start()

    def stop(self) -> None:
        self._observer.stop()
