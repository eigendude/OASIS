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
from typing import Optional

from oasis_drivers.system.device_handler import DeviceHandler
from oasis_drivers.udev.udev_process import UdevProcess


class DeviceManager:
    def __init__(self, logger: logging.Logger) -> None:
        # Construction parameters
        self._logger = logger

        # Subsystems
        self._device_handler: Optional[DeviceHandler] = None
        self._process: Optional[UdevProcess] = None

    def start(self) -> None:
        """
        Start subsystems in the order of initialization.
        """
        # Initialize subsystems
        self._device_handler = DeviceHandler(self._logger)
        self._process = UdevProcess(self._device_handler)

        # Start subsystems
        self._device_handler.start()
        self._process.start()

    def stop(self) -> None:
        """
        Stop subsystems in reverse order to respect dependencies.
        """
        if self._process is not None:
            # pyudev deinitialization is synchronous and waits for the thread
            # to exit. No need to call join().
            self._process.stop()
            self._process = None

        if self._device_handler is not None:
            self._device_handler.stop()

    def join(self) -> None:
        """
        Join subsystems in reverse order, blocking on running threads until
        they stop.
        """
        if self._device_handler is not None:
            self._device_handler.join()
            self._device_handler = None
