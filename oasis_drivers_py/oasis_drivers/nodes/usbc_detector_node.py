################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import rclpy.node
import rclpy.publisher
from std_msgs.msg import Int32

from oasis_drivers.system.usbc import usbc_max_current_ma


class UsbcDetectorNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("usbc_detector")
        # Highest USB-C device current limit in mA, or -1 when unavailable
        self._current: rclpy.publisher.Publisher = self.create_publisher(
            Int32, "usbc_current", 1
        )
        self.create_timer(1.0, self._publish)

    def _publish(self) -> None:
        current_ma: int | None = usbc_max_current_ma()
        self._current.publish(Int32(data=current_ma if current_ma is not None else -1))
