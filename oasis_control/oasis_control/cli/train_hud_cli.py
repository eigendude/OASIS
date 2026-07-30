################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""ROS entry point for the train HUD manager node."""

from typing import Optional

import rclpy

from oasis_control.nodes.train_hud_node import TrainHudNode


################################################################################
# ROS entry point
################################################################################


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)

    node: TrainHudNode = TrainHudNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
