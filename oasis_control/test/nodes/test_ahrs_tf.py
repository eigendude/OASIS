################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path
from typing import List


ROOT: Path = Path(__file__).resolve().parents[4]
PACKAGE_ROOT: Path = ROOT / "oasis_control"
PACKAGE_SRC: Path = PACKAGE_ROOT / "oasis_control"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
if "oasis_control" in sys.modules:
    module_path_raw = getattr(sys.modules["oasis_control"], "__path__", None)
    module_paths: List[str] = []
    if module_path_raw is not None:
        module_paths = list(module_path_raw)
        if str(PACKAGE_SRC) not in module_paths:
            module_paths.append(str(PACKAGE_SRC))
            sys.modules["oasis_control"].__path__ = module_paths


def _module_available(module: str) -> bool:
    root: str = module.split(".", maxsplit=1)[0]
    if importlib.util.find_spec(root) is None:
        return False
    return importlib.util.find_spec(module) is not None


REQUIRED_MODULES: list[str] = [
    "builtin_interfaces.msg",
    "geometry_msgs.msg",
    "message_filters",
    "rclpy",
    "tf2_ros",
]
missing: list[str] = [
    module for module in REQUIRED_MODULES if not _module_available(module)
]
if missing:
    raise unittest.SkipTest("ROS dependencies are unavailable: " + ", ".join(missing))

from builtin_interfaces.msg import Time as TimeMsg

from oasis_control.nodes.ahrs_node import _to_transform_stamped


class TestAhrsTf(unittest.TestCase):
    """Tests for AHRS transform helpers"""

    def test_to_transform_stamped(self) -> None:
        """_to_transform_stamped fills translation and rotation fields"""
        stamp: TimeMsg = TimeMsg(sec=123, nanosec=456)
        parent: str = "world"
        child: str = "base_link"
        translation: list[float] = [1.25, -2.5, 3.75]
        quat_wxyz: list[float] = [0.25, -0.5, 0.75, 0.333]

        msg = _to_transform_stamped(
            stamp=stamp,
            parent=parent,
            child=child,
            translation=translation,
            quat_wxyz=quat_wxyz,
        )

        self.assertEqual(msg.header.stamp, stamp)
        self.assertEqual(msg.header.frame_id, parent)
        self.assertEqual(msg.child_frame_id, child)
        self.assertAlmostEqual(msg.transform.translation.x, translation[0])
        self.assertAlmostEqual(msg.transform.translation.y, translation[1])
        self.assertAlmostEqual(msg.transform.translation.z, translation[2])
        self.assertAlmostEqual(msg.transform.rotation.x, quat_wxyz[1])
        self.assertAlmostEqual(msg.transform.rotation.y, quat_wxyz[2])
        self.assertAlmostEqual(msg.transform.rotation.z, quat_wxyz[3])
        self.assertAlmostEqual(msg.transform.rotation.w, quat_wxyz[0])
