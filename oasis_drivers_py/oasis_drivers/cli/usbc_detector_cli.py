################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import rclpy
from rclpy.executors import ExternalShutdownException

from oasis_drivers.nodes.usbc_detector_node import UsbcDetectorNode


def main(args: list[str] | None = None) -> None:
    with rclpy.init(args=args):
        node: UsbcDetectorNode = UsbcDetectorNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
