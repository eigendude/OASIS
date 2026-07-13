################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import asyncio
from collections.abc import Sequence
from datetime import datetime
from typing import Optional

from oasis_drivers.mcu.mcu_readings import AnalogReadingSample
from oasis_drivers.telemetrix.telemetrix_bridge import TelemetrixBridge
from oasis_drivers.telemetrix.telemetrix_callback import TelemetrixCallback


class _RecordingCallback(TelemetrixCallback):
    def __init__(self) -> None:
        self.equivalent_co2_ppm: int = -1
        self.tvoc_ppb: int = -1
        self.air_quality_index: Optional[int] = None

    def on_air_quality(
        self,
        timestamp: datetime,
        i2c_port: int,
        i2c_address: int,
        equivalent_co2_ppm: int,
        tvoc_ppb: int,
        air_quality_index: Optional[int],
    ) -> None:
        _ = timestamp
        _ = i2c_port
        _ = i2c_address
        self.equivalent_co2_ppm = equivalent_co2_ppm
        self.tvoc_ppb = tvoc_ppb
        self.air_quality_index = air_quality_index

    def on_analog_reading(
        self,
        timestamp: datetime,
        analog_pin: int,
        analog_value: float,
        reference_voltage: float,
    ) -> None:
        _ = timestamp
        _ = analog_pin
        _ = analog_value
        _ = reference_voltage

    def on_analog_readings(
        self, timestamp: datetime, readings: Sequence[AnalogReadingSample]
    ) -> None:
        _ = timestamp
        _ = readings

    def on_cpu_fan_rpm(self, timestamp: datetime, digital_pin: int, rpm: int) -> None:
        _ = timestamp
        _ = digital_pin
        _ = rpm

    def on_digital_reading(
        self, timestamp: datetime, digital_pin: int, digital_value: bool
    ) -> None:
        _ = timestamp
        _ = digital_pin
        _ = digital_value

    def on_imu_6_axis(
        self,
        timestamp: datetime,
        i2c_port: int,
        i2c_address: int,
        ax: int,
        ay: int,
        az: int,
        gx: int,
        gy: int,
        gz: int,
    ) -> None:
        _ = timestamp
        _ = i2c_port
        _ = i2c_address
        _ = ax
        _ = ay
        _ = az
        _ = gx
        _ = gy
        _ = gz

    def on_memory_data(
        self,
        total_ram: int,
        static_data_size: int,
        heap_size: int,
        stack_size: int,
        free_ram: int,
        free_heap: int,
    ) -> None:
        _ = total_ram
        _ = static_data_size
        _ = heap_size
        _ = stack_size
        _ = free_ram
        _ = free_heap

    def on_string_data(self, data: str) -> None:
        _ = data


def _bridge_with_callback(callback: TelemetrixCallback) -> TelemetrixBridge:
    bridge: TelemetrixBridge = TelemetrixBridge.__new__(TelemetrixBridge)
    bridge._callback = callback
    return bridge


def test_parses_equivalent_co2_and_tvoc_from_air_quality_report() -> None:
    callback = _RecordingCallback()
    bridge = _bridge_with_callback(callback)

    asyncio.run(bridge._on_air_quality([0, 0x53, 0x01, 0x90, 0x03, 0xE8]))

    assert callback.equivalent_co2_ppm == 400
    assert callback.tvoc_ppb == 1000
    assert callback.air_quality_index is None


def test_parses_optional_aqi_from_air_quality_report() -> None:
    callback = _RecordingCallback()
    bridge = _bridge_with_callback(callback)

    asyncio.run(bridge._on_air_quality([0, 0x53, 0x01, 0x90, 0x00, 0x64, 0x05]))

    assert callback.equivalent_co2_ppm == 400
    assert callback.tvoc_ppb == 100
    assert callback.air_quality_index == 5
