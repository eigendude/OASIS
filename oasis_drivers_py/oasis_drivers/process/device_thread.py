################################################################################
#
#  Copyright (C) 2019-2020 Aclima, Inc
#  This file is part of Odie Stack - https://github.com/Aclima/odie-stack
#
#  SPDX-License-Identifier: MIT
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import logging
import threading
from typing import Optional


class DeviceThread(threading.Thread):
    """
    Wraps a thread that can be aborted with a signal.
    """

    def __init__(self, logger: logging.Logger):
        # Initialize thread
        super().__init__()

        # Construction parameters
        self._log = logger

        # Threading parameters
        self._stop_event = threading.Event()

    @property
    def manufacturer(self) -> Optional[str]:
        return None

    @property
    def model(self) -> Optional[str]:
        return None

    @property
    def serial_number(self) -> Optional[str]:
        return None

    @property
    def version(self) -> Optional[str]:
        return None

    def stop(self) -> None:
        """
        Signal the stop event.
        """
        self._log.debug("Device thread received stop signal")
        self._stop_event.set()

    def should_stop(self) -> bool:
        """
        Check if the stop event has been signaled.

        :return: true if the event has been signaled, false otherwise
        """
        return self._stop_event.is_set()

    def wait_for_stop(self, timeout: Optional[float] = None) -> bool:
        """
        Wait for the thread to be stopped by a stop signal.

        :param timeout: The number of seconds to wait, or None to wait forever

        :return: true if the stop event has been signaled, false if a timeout occurred
        """
        return self._stop_event.wait(timeout)
