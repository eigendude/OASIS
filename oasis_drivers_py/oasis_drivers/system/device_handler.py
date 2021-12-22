################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import logging
from typing import Dict

from oasis_drivers.process.device_thread import DeviceThread
from oasis_drivers.udev.udev_device import UdevDevice


################################################################################
# Main class
################################################################################


class DeviceHandler(object):
    def __init__(self, logger: logging.Logger) -> None:
        # Construction parameters
        self._logger = logger

        # Device parameters
        self._devices: Dict[str, DeviceThread] = {}

    def handle_added_device(self, device: UdevDevice) -> None:
        old_devices: Dict[str, DeviceThread] = self._devices.copy()

        # Check if device was previously added
        if device.id in self._devices:
            # Ignore threads that are still running
            if not self._devices[device.id].should_stop():
                return

            # Remove devices with no running thread
            self.handle_removed_device(device)

        # The following fields, with corresponding udev names, are available to compare:
        #   - device.subsystem:     SUBSYSTEMS
        #   - device.name:          ATTRS{name}
        #   - device.product:       ATTRS{product}
        #   - device.manufacturer:  ATTRS{manufacturer}
        #   - device.vendor_id:     ATTRS{idVendor}
        #   - device.product_id:    ATTRS{idProduct}
        #   - device.serial         ATTRS{serial}

        if device.subsystem == "tty":
            # Arduino Leonardo:
            #   ID: /devices/pci0000:00/0000:00:14.0/usb1/1-4/1-4:1.0/tty/ttyACM0
            #   Path: /dev/ttyACM0
            #   Name: None
            #   Product: Arduino_Leonardo
            #   Manufacturer: Arduino_LLC
            #   VID: 2341
            #   PID: 8036
            # TODO: Case-insensitive compare
            if (device.product and "Arduino" in device.product) or (
                device.manufacturer and "Arduino" in device.manufacturer
            ):
                self._logger.info(f"Arduino connected:{device.to_string()}")

        # Update config if devices changed
        if self._devices != old_devices:
            self._update_state(self._devices, old_devices)

    def handle_changed_device(self, device: UdevDevice) -> None:
        if device.id not in self._devices:
            self.handle_added_device(device)

    def handle_removed_device(self, device: UdevDevice) -> None:
        if device.subsystem == "tty":
            # TODO: Case-insensitive compare
            if (device.product and "Arduino" in device.product) or (
                device.manufacturer and "Arduino" in device.manufacturer
            ):
                self._logger.info(f"Arduino removed:{device.to_string()}")

        if device.id in self._devices:
            old_devices = self._devices.copy()

            device_thread = self._devices[device.id]
            device_thread.stop()
            device_thread.join()
            del self._devices[device.id]

            self._update_state(self._devices, old_devices)

    def start(self) -> None:
        # Threads are started asynchronously by the Udev driver
        pass

    def stop(self) -> None:
        # Stop threads
        for device_id in self._devices:
            device_thread = self._devices[device_id]
            device_thread.stop()

    def join(self) -> None:
        # Join threads
        for device_id in self._devices:
            device_thread = self._devices[device_id]
            device_thread.join()

        # Reset state
        self._devices = {}

    def on_state_updated(self) -> None:
        """
        Handle a state update.

        This is the callback for devices when a device is updated, such as
        when it learns its serial number.
        """
        self._update_state(self._devices, self._devices)

    def _update_state(
        self, new_devices: Dict[str, DeviceThread], old_devices: Dict[str, DeviceThread]
    ) -> None:
        old_count = len(new_devices)
        new_count = len(old_devices)

        if old_count > new_count:
            self._logger.debug(
                f"Devices changed ({old_count - new_count} added), updating state"
            )
        elif old_count < new_count:
            self._logger.debug(
                f"Devices changed ({new_count - old_count} removed), updating state"
            )
        else:
            self._logger.debug(f"Devices updated ({new_count} total), updating state")

        # TODO: Publish device state
        # device_state = self._get_device_state()
