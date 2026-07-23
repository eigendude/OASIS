################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for LEGO station telemetry state handling."""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Any

import pytest
import rclpy.node
from std_msgs.msg import Float32 as Float32Msg

from oasis_control.lego_models.station_manager import MAX_MOTOR_VOLTAGE_SKEW_SECS
from oasis_control.lego_models.station_manager import MOTOR_VOLTAGE_GAIN
from oasis_control.lego_models.station_manager import MOTOR_VOLTAGE_ZERO_EPSILON
from oasis_control.lego_models.station_manager import PUBLISH_SUPPLY_VOLTAGE
from oasis_control.lego_models.station_manager import PUBLISH_TRACTION_POWER
from oasis_control.lego_models.station_manager import PUBLISH_TRACTION_VOLTAGE
from oasis_control.lego_models.station_manager import STDDEV_BETA
from oasis_control.lego_models.station_manager import SUBSCRIBE_POWER_METER_0
from oasis_control.lego_models.station_manager import SUBSCRIBE_POWER_METER_1
from oasis_control.lego_models.station_manager import VIN_GAIN
from oasis_control.lego_models.station_manager import VSS_R1
from oasis_control.lego_models.station_manager import VSS_R2
from oasis_control.lego_models.station_manager import StationManager
from oasis_control.nodes.conductor_manager_telemetrix_node import ConductorManagerNode
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import PowerMeter as PowerMeterMsg


class _Publisher:
    def __init__(self) -> None:
        self.messages: list[Any] = []

    def publish(self, message: Any) -> None:
        self.messages.append(message)


def _make_station_manager(motor_voltage_reversed: bool = False) -> StationManager:
    manager: StationManager = StationManager.__new__(StationManager)

    manager._motor_voltage_a = 0.0
    manager._motor_voltage_a_initialized = True
    manager._motor_voltage_a_timestamp_sec = 1.0
    manager._prev_motor_voltage_a = None
    manager._prev_motor_voltage_a_timestamp_sec = None
    manager._motor_voltage_b = 0.0
    manager._motor_voltage_b_initialized = True
    manager._motor_voltage_b_timestamp_sec = 1.0
    manager._prev_motor_voltage_b = None
    manager._prev_motor_voltage_b_timestamp_sec = None
    manager._motor_voltage = 0.0
    manager._motor_voltage_mu = 0.0
    manager._motor_voltage_base_var = 0.0
    manager._motor_voltage_initialized = False
    manager._motor_voltage_stddev = 0.0
    manager._motor_voltage_reversed = motor_voltage_reversed
    manager._last_motor_pwm_cmd = None
    manager._power_meter_0_current = None
    manager._power_meter_1_current = None
    manager_any: Any = manager
    manager_any._traction_voltage_pub = _Publisher()
    manager_any._traction_power_pub = _Publisher()

    return manager


def _make_initialized_station_manager() -> tuple[StationManager, rclpy.node.Node]:
    node: rclpy.node.Node = rclpy.node.Node("station_manager_test")
    manager: StationManager = StationManager(node, 0, 5, 4, 8, 7, 1, 3, 4)
    return manager, node


def _publisher(node: rclpy.node.Node, topic: str) -> Any:
    return next(publisher for publisher in node.publishers if publisher.topic == topic)


def _power_meter(
    current: float, status: int = PowerMeterMsg.STATUS_OK
) -> PowerMeterMsg:
    message: PowerMeterMsg = PowerMeterMsg()
    message.current = current
    message.status = status
    return message


def _make_pwm_station_manager() -> tuple[StationManager, _Publisher, _Publisher]:
    manager: StationManager = StationManager.__new__(StationManager)
    pwm_publisher: _Publisher = _Publisher()
    direction_publisher: _Publisher = _Publisher()

    manager._motor_pwm_pin = 5
    manager._motor_dir_pin = 6
    manager._motor_duty_cycle = 0.0
    manager._last_motor_pwm_cmd = None
    manager_any: Any = manager
    manager_any._motor_pwm_cmd_pub = pwm_publisher
    manager_any._motor_dir_cmd_pub = direction_publisher

    return manager, pwm_publisher, direction_publisher


def _update_motor_voltage(
    manager: StationManager,
    measured_motor_voltage: float,
    timestamp_sec: float = 1.0,
) -> float:
    manager._motor_voltage_a = measured_motor_voltage
    manager._motor_voltage_a_timestamp_sec = timestamp_sec
    manager._motor_voltage_b = 0.0
    manager._motor_voltage_b_timestamp_sec = timestamp_sec

    manager._update_motor_voltage()

    return manager.motor_voltage


@pytest.mark.parametrize(
    ("measured_motor_voltage", "expected_motor_voltage"),
    [
        (0.001, 0.0),
        (-0.001, 0.0),
        (0.10, 0.10 * MOTOR_VOLTAGE_GAIN),
        (-0.10, -0.10 * MOTOR_VOLTAGE_GAIN),
    ],
)
def test_motor_voltage_canonicalizes_telemetry_zero(
    measured_motor_voltage: float,
    expected_motor_voltage: float,
) -> None:
    manager: StationManager = _make_station_manager()

    motor_voltage: float = _update_motor_voltage(manager, measured_motor_voltage)

    assert motor_voltage == pytest.approx(expected_motor_voltage)
    if expected_motor_voltage == 0.0:
        assert math.copysign(1.0, motor_voltage) == 1.0

    traction_publisher: Any = manager._traction_voltage_pub
    assert len(traction_publisher.messages) == 1
    assert traction_publisher.messages[0].data == pytest.approx(expected_motor_voltage)
    if expected_motor_voltage == 0.0:
        assert math.copysign(1.0, traction_publisher.messages[0].data) == 1.0


def test_reversed_tiny_positive_measured_motor_voltage_reports_positive_zero() -> None:
    manager: StationManager = _make_station_manager(motor_voltage_reversed=True)

    motor_voltage: float = _update_motor_voltage(manager, 0.001)

    assert motor_voltage == 0.0
    assert math.copysign(1.0, motor_voltage) == 1.0


def test_reversed_voltage_sensing_negates_calibrated_differential() -> None:
    manager: StationManager = _make_station_manager(motor_voltage_reversed=True)
    manager._motor_voltage_a = 8.0
    manager._motor_voltage_b = 2.0

    manager._update_motor_voltage()

    assert manager.motor_voltage == pytest.approx(-6.0 * MOTOR_VOLTAGE_GAIN)


def test_motor_voltage_zero_epsilon_is_applied_after_calibration() -> None:
    manager: StationManager = _make_station_manager()
    raw_voltage: float = MOTOR_VOLTAGE_ZERO_EPSILON / MOTOR_VOLTAGE_GAIN - 0.0001

    assert raw_voltage > MOTOR_VOLTAGE_ZERO_EPSILON
    assert _update_motor_voltage(manager, raw_voltage) == 0.0


def test_calibrated_motor_voltage_updates_state_publication_and_statistics() -> None:
    manager: StationManager = _make_station_manager()
    first_raw_voltage: float = 4.0
    second_raw_voltage: float = 6.0
    first_calibrated_voltage: float = first_raw_voltage * MOTOR_VOLTAGE_GAIN
    second_calibrated_voltage: float = second_raw_voltage * MOTOR_VOLTAGE_GAIN

    _update_motor_voltage(manager, first_raw_voltage, timestamp_sec=1.0)
    _update_motor_voltage(manager, second_raw_voltage, timestamp_sec=2.0)

    expected_error: float = second_calibrated_voltage - first_calibrated_voltage
    expected_mean: float = first_calibrated_voltage + STDDEV_BETA * expected_error
    expected_variance: float = (
        (1.0 - STDDEV_BETA) * STDDEV_BETA * expected_error * expected_error
    )

    assert manager.motor_voltage == pytest.approx(second_calibrated_voltage)
    assert manager._motor_voltage_mu == pytest.approx(expected_mean)
    assert manager.motor_voltage_stddev == pytest.approx(math.sqrt(expected_variance))
    traction_publisher: Any = manager._traction_voltage_pub
    assert traction_publisher.messages[-1].data == pytest.approx(
        second_calibrated_voltage
    )


def test_motor_voltage_stddev_uses_canonical_zero() -> None:
    manager: StationManager = _make_station_manager()

    _update_motor_voltage(manager, 0.001, timestamp_sec=1.0)
    _update_motor_voltage(manager, -0.001, timestamp_sec=2.0)

    assert manager.motor_voltage_stddev == 0.0


def test_back_to_back_duplicate_motor_pwm_publishes_once() -> None:
    manager, pwm_publisher, _ = _make_pwm_station_manager()

    manager.set_motor_pwm(0.5, False)
    manager.set_motor_pwm(0.5, False)

    assert len(pwm_publisher.messages) == 1
    assert pwm_publisher.messages[0].digital_pin == 5
    assert pwm_publisher.messages[0].duty_cycle == pytest.approx(0.5)
    assert manager.motor_duty_cycle == pytest.approx(0.5)


def test_motor_direction_side_effect_resets_duplicate_pwm_cache() -> None:
    manager, pwm_publisher, direction_publisher = _make_pwm_station_manager()

    manager.set_motor_pwm(0.5, False)
    manager.set_motor_direction(True)
    manager.set_motor_pwm(0.5, True)

    assert len(pwm_publisher.messages) == 2
    assert len(direction_publisher.messages) == 1
    assert manager.motor_duty_cycle == pytest.approx(-0.5)


def test_opposite_reverse_motor_pwm_republishes_same_magnitude() -> None:
    manager, pwm_publisher, _ = _make_pwm_station_manager()

    manager.set_motor_pwm(0.5, False)
    manager.set_motor_pwm(0.5, True)

    assert len(pwm_publisher.messages) == 2
    assert pwm_publisher.messages[0].duty_cycle == pytest.approx(0.5)
    assert pwm_publisher.messages[1].duty_cycle == pytest.approx(0.5)
    assert manager.motor_duty_cycle == pytest.approx(-0.5)


def test_measurement_publishers_use_float32_and_sensor_data_qos() -> None:
    _, node = _make_initialized_station_manager()

    supply_publisher: Any = _publisher(node, PUBLISH_SUPPLY_VOLTAGE)
    traction_publisher: Any = _publisher(node, PUBLISH_TRACTION_VOLTAGE)
    power_publisher: Any = _publisher(node, PUBLISH_TRACTION_POWER)

    assert supply_publisher.msg_type is Float32Msg
    assert traction_publisher.msg_type is Float32Msg
    assert power_publisher.msg_type is Float32Msg
    assert supply_publisher.qos_profile is rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
    assert (
        traction_publisher.qos_profile is rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
    )
    assert power_publisher.qos_profile is rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value


def test_power_meter_subscribers_use_sensor_data_qos() -> None:
    _, node = _make_initialized_station_manager()
    subscriptions: dict[str, Any] = {
        subscription.topic: subscription for subscription in node.subscriptions
    }

    for topic in (SUBSCRIBE_POWER_METER_0, SUBSCRIBE_POWER_METER_1):
        assert (
            subscriptions[topic].qos_profile
            is rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )


def test_traction_power_waits_for_both_currents_and_uses_measured_voltage() -> None:
    manager: StationManager = _make_station_manager()
    power_publisher: Any = manager._traction_power_pub

    manager._on_power_meter_0(_power_meter(1.25))
    _update_motor_voltage(manager, 7.0)
    assert power_publisher.messages == []

    manager._on_power_meter_1(_power_meter(0.75, PowerMeterMsg.STATUS_STALE))
    _update_motor_voltage(manager, 7.0)

    expected_voltage: float = 7.0 * MOTOR_VOLTAGE_GAIN
    assert len(power_publisher.messages) == 1
    assert power_publisher.messages[0].data == pytest.approx(
        expected_voltage * (1.25 + 0.75)
    )


def test_reverse_traction_voltage_produces_positive_power() -> None:
    manager: StationManager = _make_station_manager()
    manager._on_power_meter_0(_power_meter(1.5))
    manager._on_power_meter_1(_power_meter(0.5))

    _update_motor_voltage(manager, -4.0)

    power_publisher: Any = manager._traction_power_pub
    assert power_publisher.messages[0].data == pytest.approx(
        abs(-4.0 * MOTOR_VOLTAGE_GAIN) * (1.5 + 0.5)
    )


def test_zero_traction_power_is_positive_zero() -> None:
    manager: StationManager = _make_station_manager()
    manager._on_power_meter_0(_power_meter(1.0))
    manager._on_power_meter_1(_power_meter(2.0))

    _update_motor_voltage(manager, 0.0)

    power_publisher: Any = manager._traction_power_pub
    power: float = power_publisher.messages[0].data
    assert power == 0.0
    assert math.copysign(1.0, power) == 1.0


@pytest.mark.parametrize(
    ("status", "current"),
    [
        (PowerMeterMsg.STATUS_ERROR, 1.0),
        (PowerMeterMsg.STATUS_DISCONNECTED, 1.0),
        (PowerMeterMsg.STATUS_OK, math.nan),
        (PowerMeterMsg.STATUS_STALE, math.inf),
        (PowerMeterMsg.STATUS_OK, -1.0),
    ],
)
def test_invalid_current_suppresses_power_until_valid_sample(
    status: int, current: float
) -> None:
    manager: StationManager = _make_station_manager()
    manager._on_power_meter_0(_power_meter(1.0))
    manager._on_power_meter_1(_power_meter(2.0))
    _update_motor_voltage(manager, 3.0)
    power_publisher: Any = manager._traction_power_pub
    assert len(power_publisher.messages) == 1
    power_publisher.messages.clear()

    manager._on_power_meter_1(_power_meter(current, status))

    _update_motor_voltage(manager, 3.0)
    assert power_publisher.messages == []

    manager._on_power_meter_1(_power_meter(2.0))
    _update_motor_voltage(manager, 3.0)
    assert len(power_publisher.messages) == 1


def test_supply_adc_update_publishes_only_converted_supply_voltage() -> None:
    manager, node = _make_initialized_station_manager()
    supply_publisher: Any = _publisher(node, PUBLISH_SUPPLY_VOLTAGE)
    reading: AnalogReadingMsg = AnalogReadingMsg()
    reading.analog_pin = 0
    reading.analog_value = 0.5
    reading.reference_voltage = 5.0

    manager._handle_analog_reading(reading, 1.0)

    expected_voltage: float = (
        reading.analog_value * reading.reference_voltage * (VSS_R1 + VSS_R2) / VSS_R2
    )
    expected_voltage *= VIN_GAIN
    assert len(supply_publisher.messages) == 1
    assert supply_publisher.messages[0].data == pytest.approx(expected_voltage)

    reading.analog_pin = 1
    manager._handle_analog_reading(reading, 1.1)
    assert len(supply_publisher.messages) == 1


def test_incomplete_and_stale_motor_voltage_pairs_do_not_publish() -> None:
    manager, node = _make_initialized_station_manager()
    traction_publisher: Any = _publisher(node, PUBLISH_TRACTION_VOLTAGE)

    manager._update_motor_voltage_channel_a(8.0, 1.0)
    assert traction_publisher.messages == []

    manager._update_motor_voltage_channel_b(
        2.0,
        1.0 + MAX_MOTOR_VOLTAGE_SKEW_SECS + 0.01,
    )
    assert traction_publisher.messages == []


@pytest.mark.parametrize(
    ("motor_voltage_reversed", "expected_voltage"),
    [
        (False, 6.0 * MOTOR_VOLTAGE_GAIN),
        (True, -6.0 * MOTOR_VOLTAGE_GAIN),
    ],
)
def test_completed_motor_voltage_pair_publishes_signed_value(
    motor_voltage_reversed: bool,
    expected_voltage: float,
) -> None:
    manager, node = _make_initialized_station_manager()
    manager._motor_voltage_reversed = motor_voltage_reversed
    traction_publisher: Any = _publisher(node, PUBLISH_TRACTION_VOLTAGE)

    manager._update_motor_voltage_channel_a(8.0, 1.0)
    manager._update_motor_voltage_channel_b(2.0, 1.0)

    assert len(traction_publisher.messages) == 1
    assert traction_publisher.messages[0].data == pytest.approx(expected_voltage)


def test_periodic_state_preserves_fields_without_republishing_scalar_topics() -> None:
    node: ConductorManagerNode = ConductorManagerNode.__new__(ConductorManagerNode)
    conductor_publisher: _Publisher = _Publisher()
    supply_publisher: _Publisher = _Publisher()
    traction_publisher: _Publisher = _Publisher()
    power_publisher: _Publisher = _Publisher()
    node_any: Any = node
    node_any._conductor_state_pub = conductor_publisher
    node_any._station_manager = SimpleNamespace(
        supply_voltage=12.3,
        supply_voltage_stddev=0.2,
        motor_voltage=-4.5,
        motor_voltage_stddev=0.3,
        motor_duty_cycle=-0.4,
        motor_current=1.2,
        motor_ff1_count=7,
        motor_ff2_count=8,
        _supply_voltage_pub=supply_publisher,
        _traction_voltage_pub=traction_publisher,
        _traction_power_pub=power_publisher,
    )
    node_any._mcu_memory_manager = SimpleNamespace(
        total_ram=4096,
        ram_utilization=0.25,
    )

    node._publish_state()

    assert len(conductor_publisher.messages) == 1
    state: Any = conductor_publisher.messages[0]
    assert state.supply_voltage == pytest.approx(12.3)
    assert state.supply_voltage_stddev == pytest.approx(0.2)
    assert state.motor_voltage == pytest.approx(-4.5)
    assert state.motor_voltage_stddev == pytest.approx(0.3)
    assert state.duty_cycle == pytest.approx(-0.4)
    assert state.motor_current == pytest.approx(1.2)
    assert state.motor_ff1_count == 7
    assert state.motor_ff2_count == 8
    assert supply_publisher.messages == []
    assert traction_publisher.messages == []
    assert power_publisher.messages == []
