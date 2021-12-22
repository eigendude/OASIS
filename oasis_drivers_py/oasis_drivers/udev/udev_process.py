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
from typing_extensions import Protocol

from oasis_drivers.udev.udev_device import UdevDevice
from oasis_drivers.udev.udev_monitor import UdevMonitor
from oasis_drivers.udev.udev_monitor_observer import UdevMonitorObserver


class SupportsDeviceCallbacks(Protocol):
    def handle_added_device(self, device: UdevDevice) -> None:
        ...

    def handle_changed_device(self, device: UdevDevice) -> None:
        ...

    def handle_removed_device(self, device: UdevDevice) -> None:
        ...


class UdevProcess(object):
    """
    Create an asynchronous event monitor and handles udev events.
    """

    def __init__(self, device_manager: SupportsDeviceCallbacks) -> None:
        # Construction parameters
        self._device_manager = device_manager

        # udev parameters
        self._udev_context = pyudev.Context()
        self._udev_monitor = UdevMonitor(self._udev_context)
        self._observer = UdevMonitorObserver(self, self._udev_monitor)

    def start(self) -> None:
        """
        Start the event monitors and begins handling events.
        """
        # Dispatch events for current devices
        for device in self._udev_context.list_devices():
            self.handle_event("add", device)

        self._observer.start()

    def stop(self) -> None:
        """
        Stop the event monitors and finishes handling events.
        """
        self._observer.stop()

    def handle_event(self, action: str, udev_device: pyudev.Device) -> None:
        """
        Handle a udev action.

        :param action: The udev action
        :param udev_device: The pyudev device being acted upon.
        """
        device: UdevDevice = UdevDevice.from_udev_device(udev_device)

        if not device.is_valid():
            return

        # Dispatch event
        try:
            {
                "add": self._device_manager.handle_added_device,
                "change": self._device_manager.handle_changed_device,
                "remove": self._device_manager.handle_removed_device,
            }[action](device)
        except KeyError:
            pass
