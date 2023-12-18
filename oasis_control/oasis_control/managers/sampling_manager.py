################################################################################
#
#  Copyright (C) 2022-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a mcu that reports memory statistics
#

import asyncio

import rclpy.client
import rclpy.node

from oasis_msgs.srv import SetSamplingInterval as SetSamplingIntervalSvc


################################################################################
# ROS parameters
################################################################################


# Service clients
CLIENT_SET_SAMPLING_INTERVAL = "set_sampling_interval"


################################################################################
# Manager
################################################################################


class SamplingManager:
    """
    A manager for sensing and reporting MCU memory
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        """
        Initialize resources.
        """
        # Initialize manager state
        self._node: rclpy.node.Node = node
        self._logger = node.get_logger()

        # Service clients
        self._set_sampling_interval_client: rclpy.client.Client = (
            self._node.create_client(
                srv_type=SetSamplingIntervalSvc, srv_name=CLIENT_SET_SAMPLING_INTERVAL
            )
        )

    def initialize(self, sampling_interval_ms: int) -> bool:
        self._logger.debug("Waiting for sampling services")
        self._logger.debug("  - Waiting for set_sampling_interval...")
        self._set_sampling_interval_client.wait_for_service()

        self._logger.debug("Starting sampling configuration")

        # Sampling interval
        self._logger.debug(f"Setting sampling interval to {sampling_interval_ms} ms")
        if not self._set_sampling_interval(sampling_interval_ms):
            return False

        self._logger.info("Sampling manager initialized successfully")

        return True

    def _set_sampling_interval(self, sampling_interval_ms: int) -> bool:
        # Create message
        set_sampling_interval_svc = SetSamplingIntervalSvc.Request()
        set_sampling_interval_svc.sampling_interval_ms = sampling_interval_ms

        # Call service
        future: asyncio.Future = self._set_sampling_interval_client.call_async(
            set_sampling_interval_svc
        )

        # Wait for result
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._logger.error(f"Exception while calling service: {future.exception()}")
            return False

        return True
