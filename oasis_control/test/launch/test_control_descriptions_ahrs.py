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


if not _module_available("launch.launch_description"):
    raise unittest.SkipTest("ROS launch is unavailable")
if not _module_available("launch_ros.actions"):
    raise unittest.SkipTest("ROS launch_ros is unavailable")

from launch.launch_description import LaunchDescription
from launch_ros.actions import Node

from oasis_control.launch.control_descriptions import ControlDescriptions


class TestControlDescriptionsAhrs(unittest.TestCase):
    """Tests for AHRS launch descriptions"""

    def test_add_ahrs_node_remappings(self) -> None:
        """add_ahrs_node remaps state and diagnostics topics"""
        ld: LaunchDescription = LaunchDescription()
        ControlDescriptions.add_ahrs_node(ld, "falcon")

        nodes: list[Node] = [
            entity for entity in ld.entities if isinstance(entity, Node)
        ]
        self.assertEqual(len(nodes), 1)
        remappings: list[tuple[str, str]] = list(nodes[0].remappings)

        self.assertIn(("ahrs/state", "falcon/ahrs/state"), remappings)
        self.assertIn(("ahrs/diagnostics", "falcon/ahrs/diagnostics"), remappings)
