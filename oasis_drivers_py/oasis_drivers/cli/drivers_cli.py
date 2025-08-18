################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Launch multiple driver nodes in a single process.

This CLI combines the individual driver entry points so that a single Python
interpreter can host all driver nodes.  This can reduce CPU and memory overhead
on resource constrained systems like the Raspberry Pi.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor

from oasis_drivers.nodes.display_server_node import DisplayServerNode
from oasis_drivers.nodes.firmata_bridge_node import FirmataBridgeNode
from oasis_drivers.nodes.serial_scanner_node import SerialScannerNode
from oasis_drivers.nodes.system_monitor_node import SystemMonitorNode
from oasis_drivers.nodes.telemetrix_bridge_node import TelemetrixBridgeNode
from oasis_drivers.nodes.ups_server_node import UpsServerNode
from oasis_drivers.nodes.wol_server_node import WolServerNode


################################################################################
# ROS entry point
################################################################################


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    nodes = [
        DisplayServerNode(),
        FirmataBridgeNode(),
        SerialScannerNode(),
        SystemMonitorNode(),
        TelemetrixBridgeNode(),
        UpsServerNode(),
        WolServerNode(),
    ]

    # Call optional initialize() methods
    for node in nodes:
        init = getattr(node, "initialize", None)
        if callable(init):
            try:
                init()
            except Exception as err:  # pragma: no cover - log and continue
                node.get_logger().error(f"Initialization failed: {err}")

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            stop = getattr(node, "stop", None)
            if callable(stop):
                try:
                    stop()
                except Exception:  # pragma: no cover - best effort shutdown
                    pass

    rclpy.shutdown()
