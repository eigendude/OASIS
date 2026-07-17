################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from collections.abc import Callable

import pytest
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.descriptions import ComposableNode

from oasis_drivers.launch.driver_descriptions import DriverDescriptions


def test_power_meter_node_uses_root_namespace_and_configured_remappings() -> None:
    composable_nodes: list[ComposableNode] = []
    parameters: dict[str, object] = {"power_meter_ids": ["input_meter", "load_meter"]}

    DriverDescriptions.add_power_meter_node(
        composable_nodes,
        "airlab",
        parameters,
    )

    assert parameters == {"power_meter_ids": ["input_meter", "load_meter"]}
    assert len(composable_nodes) == 1
    node: ComposableNode = composable_nodes[0]
    context: LaunchContext = LaunchContext()
    assert node.node_namespace is not None
    assert perform_substitutions(context, node.node_namespace) == "oasis"
    assert node.node_name is not None
    assert perform_substitutions(context, node.node_name) == "power_meter_airlab"
    assert node.remappings is not None
    remappings: list[tuple[str, str]] = [
        (
            perform_substitutions(context, list(source)),
            perform_substitutions(context, list(destination)),
        )
        for source, destination in node.remappings
    ]
    assert remappings == [
        ("input_meter", "airlab/input_meter"),
        ("load_meter", "airlab/load_meter"),
    ]


def test_ssd1305_display_node_uses_root_namespace_and_configured_remappings() -> None:
    composable_nodes: list[ComposableNode] = []
    parameters: dict[str, object] = {"i2c_device": "/dev/i2c-1"}

    DriverDescriptions.add_ssd1305_display_node(
        composable_nodes,
        "abn_007000",
        parameters,
    )

    assert len(composable_nodes) == 1
    node: ComposableNode = composable_nodes[0]
    context: LaunchContext = LaunchContext()
    assert node.node_namespace is not None
    assert perform_substitutions(context, node.node_namespace) == "oasis"
    assert node.node_name is not None
    assert (
        perform_substitutions(context, node.node_name) == "ssd1305_display_abn_007000"
    )
    assert node.remappings is not None
    remappings: list[tuple[str, str]] = [
        (
            perform_substitutions(context, list(source)),
            perform_substitutions(context, list(destination)),
        )
        for source, destination in node.remappings
    ]
    assert remappings == [
        ("display/image", "abn_007000/display/image"),
        ("display/set_enabled", "abn_007000/display/set_enabled"),
        ("display/clear", "abn_007000/display/clear"),
        ("display/set_contrast", "abn_007000/display/set_contrast"),
        ("display/set_invert", "abn_007000/display/set_invert"),
    ]


@pytest.mark.parametrize(
    ("parameters", "expected_exception", "expected_message"),
    [
        ({}, TypeError, "power_meter_ids must be a list of strings"),
        (
            {"power_meter_ids": "input_meter"},
            TypeError,
            "power_meter_ids must be a list of strings",
        ),
        (
            {"power_meter_ids": ["input_meter", 1]},
            TypeError,
            "power_meter_ids must contain only strings",
        ),
        (
            {"power_meter_ids": [""]},
            ValueError,
            "power_meter_ids must not contain empty IDs",
        ),
        (
            {"power_meter_ids": ["duplicate", "duplicate"]},
            ValueError,
            "power_meter_ids contains duplicate ID 'duplicate'",
        ),
    ],
)
def test_power_meter_node_rejects_invalid_ids(
    parameters: dict[str, object],
    expected_exception: type[Exception],
    expected_message: str,
) -> None:
    add_node: Callable[[], None] = lambda: DriverDescriptions.add_power_meter_node(
        [],
        "airlab",
        parameters,
    )

    with pytest.raises(expected_exception, match=expected_message):
        add_node()
