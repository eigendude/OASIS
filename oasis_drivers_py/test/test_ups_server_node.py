################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

# mypy: disable-error-code=import-not-found

"""Tests for the UPS status to battery-state publisher."""

from __future__ import annotations

import math
from typing import Any
from typing import Protocol
from typing import cast

import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import BatteryState as BatteryStateMsg

from oasis_drivers.nodes.ups_server_node import UpsServerNode
from oasis_msgs.msg import UPSStatus as UPSStatusMsg


class _RecordingPublisher(Protocol):
    messages: list[Any]


def _messages(publisher: object) -> list[Any]:
    """Return messages captured by the test publisher stub"""
    return cast(_RecordingPublisher, publisher).messages


@pytest.fixture(scope="module", autouse=True)
def rclpy_context() -> Any:
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_publishes_battery_topic_with_state_qos() -> None:
    node: UpsServerNode = UpsServerNode(start_thread=False)

    try:
        assert node._ups_status_publisher.topic == "ups_status"
        assert node._battery_state_publisher.topic == "battery"
        assert (
            node._battery_state_publisher.qos_profile.reliability
            is ReliabilityPolicy.RELIABLE
        )
        assert (
            node._battery_state_publisher.qos_profile.durability
            is DurabilityPolicy.TRANSIENT_LOCAL
        )
        assert (
            node._battery_state_publisher.qos_profile.history is HistoryPolicy.KEEP_LAST
        )
        assert node._battery_state_publisher.qos_profile.depth == 1
    finally:
        node.stop()


def test_republishes_discharging_ups_status_as_battery_state() -> None:
    node: UpsServerNode = UpsServerNode(start_thread=False)
    ups_status: UPSStatusMsg = make_ups_status(
        serial_number="CXXNU7006846",
        battery_voltage=24.8,
        battery_current=2.5,
        battery_temperature=31.0,
        battery_charge=85,
        status="OB DISCHRG",
    )

    try:
        node._publish_status_messages(ups_status)

        assert _messages(node._ups_status_publisher)[-1] is ups_status

        battery_state: BatteryStateMsg = _messages(node._battery_state_publisher)[-1]
        assert battery_state.voltage == 24.8
        assert battery_state.current == -2.5
        assert battery_state.temperature == 31.0
        assert math.isnan(battery_state.charge)
        assert math.isnan(battery_state.capacity)
        assert math.isnan(battery_state.design_capacity)
        assert battery_state.percentage == 0.85
        assert (
            battery_state.power_supply_status
            == BatteryStateMsg.POWER_SUPPLY_STATUS_DISCHARGING
        )
        assert (
            battery_state.power_supply_health
            == BatteryStateMsg.POWER_SUPPLY_HEALTH_GOOD
        )
        assert (
            battery_state.power_supply_technology
            == BatteryStateMsg.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        )
        assert battery_state.present is True
        assert battery_state.cell_voltage == []
        assert battery_state.cell_temperature == []
        assert battery_state.location == "ups"
        assert battery_state.serial_number == "CXXNU7006846"
    finally:
        node.stop()


def test_reports_charging_current_and_lithium_ion_technology() -> None:
    node: UpsServerNode = UpsServerNode(start_thread=False)
    ups_status: UPSStatusMsg = make_ups_status(
        battery_voltage=25.1,
        battery_current=1.2,
        battery_charge=91,
        status="OL CHRG",
        battery_type="Li-ion",
    )

    try:
        node._publish_status_messages(ups_status)

        battery_state: BatteryStateMsg = _messages(node._battery_state_publisher)[-1]
        assert battery_state.current == 1.2
        assert (
            battery_state.power_supply_status
            == BatteryStateMsg.POWER_SUPPLY_STATUS_CHARGING
        )
        assert (
            battery_state.power_supply_technology
            == BatteryStateMsg.POWER_SUPPLY_TECHNOLOGY_LION
        )
    finally:
        node.stop()


def test_estimates_discharging_current_from_load_when_current_is_unknown() -> None:
    node: UpsServerNode = UpsServerNode(start_thread=False)
    ups_status: UPSStatusMsg = make_ups_status(
        battery_voltage=24.0,
        battery_current=math.nan,
        load=48.0,
        status="OB",
    )

    try:
        node._publish_status_messages(ups_status)

        battery_state: BatteryStateMsg = _messages(node._battery_state_publisher)[-1]
        assert battery_state.current == -2.0
    finally:
        node.stop()


def test_online_full_battery_reports_full_status() -> None:
    node: UpsServerNode = UpsServerNode(start_thread=False)
    ups_status: UPSStatusMsg = make_ups_status(
        battery_voltage=27.0,
        battery_current=math.nan,
        battery_charge=100,
        status="OL",
    )

    try:
        node._publish_status_messages(ups_status)

        battery_state: BatteryStateMsg = _messages(node._battery_state_publisher)[-1]
        assert math.isnan(battery_state.current)
        assert (
            battery_state.power_supply_status
            == BatteryStateMsg.POWER_SUPPLY_STATUS_FULL
        )
    finally:
        node.stop()


def test_replace_battery_status_reports_dead_health() -> None:
    node: UpsServerNode = UpsServerNode(start_thread=False)
    ups_status: UPSStatusMsg = make_ups_status(
        battery_voltage=0.0,
        status="OL RB",
    )

    try:
        node._publish_status_messages(ups_status)

        battery_state: BatteryStateMsg = _messages(node._battery_state_publisher)[-1]
        assert (
            battery_state.power_supply_health
            == BatteryStateMsg.POWER_SUPPLY_HEALTH_DEAD
        )
    finally:
        node.stop()


def make_ups_status(
    serial_number: str = "",
    input_voltage: float = 120.0,
    output_voltage: float = 120.0,
    battery_voltage: float = math.nan,
    battery_current: float = math.nan,
    battery_temperature: float = math.nan,
    battery_charge: int = 0,
    load: float = 0.0,
    status: str = "",
    battery_type: str = "",
) -> UPSStatusMsg:
    return UPSStatusMsg(
        manufacturer="CPS",
        model="CP1500PFCLCDa",
        serial_number=serial_number,
        input_voltage=input_voltage,
        output_voltage=output_voltage,
        battery_voltage=battery_voltage,
        battery_current=battery_current,
        battery_temperature=battery_temperature,
        battery_charge=battery_charge,
        battery_runtime=0,
        load=load,
        load_total=900.0,
        status=status,
        battery_type=battery_type,
    )
