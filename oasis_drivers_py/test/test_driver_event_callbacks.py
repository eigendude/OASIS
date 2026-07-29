################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for driver-node ROS topic event callback configuration."""

import ast
from pathlib import Path
from typing import Any

from oasis_drivers.ros.event_callbacks import publisher_event_callbacks
from oasis_drivers.ros.event_callbacks import subscription_event_callbacks


PACKAGE_ROOT: Path = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT: Path = PACKAGE_ROOT.parent
TELEMETRIX_BRIDGE_MODULE: Path = (
    PACKAGE_ROOT / "oasis_drivers/nodes/telemetrix_bridge_node.py"
)
SYSTEM_MONITOR_MODULE: Path = (
    PACKAGE_ROOT / "oasis_drivers/nodes/system_monitor_node.py"
)
FALCON_TOPIC_MODULES: dict[Path, tuple[int, int]] = {
    TELEMETRIX_BRIDGE_MODULE: (10, 4),
    (REPOSITORY_ROOT / "oasis_control/oasis_control/nodes/ahrs_speedometer_node.py"): (
        1,
        3,
    ),
    (REPOSITORY_ROOT / "oasis_control/oasis_control/nodes/engineer_manager_node.py"): (
        1,
        1,
    ),
    (REPOSITORY_ROOT / "oasis_control/oasis_control/mcu/mcu_memory_manager.py"): (0, 1),
}


def test_callback_factories_disable_defaults_and_return_fresh_containers() -> None:
    publisher_callbacks = publisher_event_callbacks()
    other_publisher_callbacks = publisher_event_callbacks()
    subscription_callbacks = subscription_event_callbacks()
    other_subscription_callbacks = subscription_event_callbacks()

    assert publisher_callbacks.use_default_callbacks is False
    assert subscription_callbacks.use_default_callbacks is False
    assert publisher_callbacks is not other_publisher_callbacks
    assert subscription_callbacks is not other_subscription_callbacks


def test_callback_factories_preserve_explicit_callbacks() -> None:
    def on_incompatible_qos(event: Any) -> None:
        del event

    publisher_callbacks = publisher_event_callbacks(
        incompatible_qos=on_incompatible_qos
    )
    subscription_callbacks = subscription_event_callbacks(
        incompatible_qos=on_incompatible_qos
    )

    assert publisher_callbacks.incompatible_qos is on_incompatible_qos
    assert subscription_callbacks.incompatible_qos is on_incompatible_qos


def test_telemetrix_topic_entities_pass_fresh_event_callback_factories() -> None:
    assert _topic_factory_counts(TELEMETRIX_BRIDGE_MODULE) == (10, 4)


def test_falcon_python_topic_module_inventory_uses_callback_factories() -> None:
    assert SYSTEM_MONITOR_MODULE not in FALCON_TOPIC_MODULES

    for module_path, expected_counts in FALCON_TOPIC_MODULES.items():
        assert _topic_factory_counts(module_path) == expected_counts


def test_system_monitor_remains_explicitly_excluded() -> None:
    module: ast.Module = ast.parse(SYSTEM_MONITOR_MODULE.read_text())
    publisher_calls: list[ast.Call] = [
        node
        for node in ast.walk(module)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_publisher"
    ]

    assert len(publisher_calls) == 1
    assert all(
        keyword.arg != "event_callbacks" for keyword in publisher_calls[0].keywords
    )


def _topic_factory_counts(module_path: Path) -> tuple[int, int]:
    module: ast.Module = ast.parse(module_path.read_text())
    publisher_calls: int = 0
    subscription_calls: int = 0

    for node in ast.walk(module):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue

        expected_factory: str | None = None
        if node.func.attr == "create_publisher":
            publisher_calls += 1
            expected_factory = publisher_event_callbacks.__name__
        elif node.func.attr == "create_subscription":
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

    return publisher_calls, subscription_calls
