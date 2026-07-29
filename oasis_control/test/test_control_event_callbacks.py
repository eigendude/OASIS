################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for control-node ROS topic event callback configuration."""

import ast
from pathlib import Path

from oasis_drivers.ros.event_callbacks import publisher_event_callbacks
from oasis_drivers.ros.event_callbacks import subscription_event_callbacks


PACKAGE_ROOT: Path = Path(__file__).resolve().parents[1]

AFFECTED_MODULES: tuple[Path, ...] = (
    PACKAGE_ROOT / "oasis_control/nodes/conductor_manager_telemetrix_node.py",
    PACKAGE_ROOT / "oasis_control/lego_models/station_manager.py",
    PACKAGE_ROOT / "oasis_control/lego_models/helipad_manager.py",
    PACKAGE_ROOT / "oasis_control/input/station_input.py",
    PACKAGE_ROOT / "oasis_control/mcu/mcu_memory_manager.py",
)


def test_conductor_topic_entities_pass_fresh_event_callback_factories() -> None:
    publisher_calls: int = 0
    subscription_calls: int = 0

    for module_path in AFFECTED_MODULES:
        module: ast.Module = ast.parse(module_path.read_text())
        for node in ast.walk(module):
            if not isinstance(node, ast.Call):
                continue

            function_name: str | None = None
            if isinstance(node.func, ast.Attribute):
                function_name = node.func.attr

            expected_factory: str | None = None
            if function_name == "create_publisher":
                publisher_calls += 1
                expected_factory = publisher_event_callbacks.__name__
            elif function_name == "create_subscription":
                subscription_calls += 1
                expected_factory = subscription_event_callbacks.__name__

            if expected_factory is None:
                continue

            event_keywords: list[ast.keyword] = [
                keyword for keyword in node.keywords if keyword.arg == "event_callbacks"
            ]
            assert len(event_keywords) == 1
            factory_call: ast.AST = event_keywords[0].value
            assert isinstance(factory_call, ast.Call)
            assert isinstance(factory_call.func, ast.Name)
            assert factory_call.func.id == expected_factory

    assert publisher_calls == 6
    assert subscription_calls == 10
