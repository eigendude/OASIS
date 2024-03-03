################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# ROS node to scan for serial port devices using pyserial
#

import rclpy

from oasis_drivers.nodes.serial_scanner_node import SerialScannerNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    scanner = SerialScannerNode()

    try:
        rclpy.spin(scanner)
    except KeyboardInterrupt:
        pass
    finally:
        scanner.stop()
