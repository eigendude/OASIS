################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Factories for ROS topic event callback containers."""

from typing import Any

from rclpy.event_handler import PublisherEventCallbacks
from rclpy.event_handler import SubscriptionEventCallbacks


def publisher_event_callbacks(**callbacks: Any) -> PublisherEventCallbacks:
    """Create publisher callbacks without rclpy's automatic warning handlers."""
    callbacks["use_default_callbacks"] = False
    return PublisherEventCallbacks(**callbacks)


def subscription_event_callbacks(**callbacks: Any) -> SubscriptionEventCallbacks:
    """Create subscription callbacks without rclpy's automatic warning handlers."""
    callbacks["use_default_callbacks"] = False
    return SubscriptionEventCallbacks(**callbacks)
