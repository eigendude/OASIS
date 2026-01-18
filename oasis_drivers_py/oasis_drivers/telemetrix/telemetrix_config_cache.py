################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

import threading
from typing import Dict
from typing import Optional
from typing import Protocol
from typing import Set
from typing import Tuple

from oasis_drivers.telemetrix.telemetrix_bridge import TelemetrixBridge
from oasis_drivers.telemetrix.telemetrix_types import AnalogMode
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode


class TelemetrixLogger(Protocol):
    def info(self, message: str) -> None:
        """Log info"""
        ...

    def warning(self, message: str) -> None:
        """Log warning"""
        ...


class TelemetrixConfigCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sampling_interval_ms: Optional[int] = None
        self._memory_reporting_interval_ms: Optional[int] = None
        self._cpu_fan_sampling_interval_ms: Optional[int] = None
        self._digital_modes: Dict[int, DigitalMode] = {}
        self._analog_modes: Dict[int, AnalogMode] = {}
        self._i2c_ports_started: Set[int] = set()
        self._ccs811_devices: Set[Tuple[int, int]] = set()
        self._mpu6050_devices: Set[Tuple[int, int]] = set()

    def record_sampling_interval(self, ms: int) -> None:
        with self._lock:
            self._sampling_interval_ms = ms

    def record_memory_reporting_interval(self, ms: int) -> None:
        with self._lock:
            self._memory_reporting_interval_ms = ms

    def record_cpu_fan_sampling_interval(self, ms: int) -> None:
        with self._lock:
            self._cpu_fan_sampling_interval_ms = ms

    def record_digital_mode(self, pin: int, mode: DigitalMode) -> None:
        with self._lock:
            self._digital_modes[pin] = mode

    def record_analog_mode(self, pin: int, mode: AnalogMode) -> None:
        with self._lock:
            self._analog_modes[pin] = mode

    def record_i2c_begin(self, port: int) -> None:
        with self._lock:
            self._i2c_ports_started.add(port)

    def record_ccs811_begin(self, port: int, address: int) -> None:
        with self._lock:
            self._ccs811_devices.add((port, address))

    def record_ccs811_end(self, port: int, address: int) -> None:
        with self._lock:
            self._ccs811_devices.discard((port, address))

    def record_mpu6050_begin(self, port: int, address: int) -> None:
        with self._lock:
            self._mpu6050_devices.add((port, address))

    def record_mpu6050_end(self, port: int, address: int) -> None:
        with self._lock:
            self._mpu6050_devices.discard((port, address))

    def replay(self, bridge: TelemetrixBridge, logger: TelemetrixLogger) -> None:
        with self._lock:
            sampling_interval_ms: Optional[int] = self._sampling_interval_ms
            memory_reporting_interval_ms: Optional[int] = (
                self._memory_reporting_interval_ms
            )
            cpu_fan_sampling_interval_ms: Optional[int] = (
                self._cpu_fan_sampling_interval_ms
            )
            i2c_ports_started: Set[int] = set(self._i2c_ports_started)
            digital_modes: Dict[int, DigitalMode] = dict(self._digital_modes)
            analog_modes: Dict[int, AnalogMode] = dict(self._analog_modes)
            ccs811_devices: Set[Tuple[int, int]] = set(self._ccs811_devices)
            mpu6050_devices: Set[Tuple[int, int]] = set(self._mpu6050_devices)

        logger.info(f"Replaying Telemetrix config: {self.dump()}")
        failures: bool = False

        if memory_reporting_interval_ms is not None:
            try:
                bridge.set_memory_reporting_interval(memory_reporting_interval_ms)
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay memory reporting interval "
                    f"{memory_reporting_interval_ms}: {exc!r}"
                )

        if sampling_interval_ms is not None:
            try:
                bridge.set_sampling_interval(sampling_interval_ms)
            except Exception as exc:
                failures = True
                logger.warning(
                    f"Failed to replay sampling interval {sampling_interval_ms}: {exc!r}"
                )

        if cpu_fan_sampling_interval_ms is not None:
            try:
                bridge.set_cpu_fan_sampling_interval(cpu_fan_sampling_interval_ms)
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay CPU fan sampling interval "
                    f"{cpu_fan_sampling_interval_ms}: {exc!r}"
                )

        for port in sorted(i2c_ports_started):
            try:
                bridge.i2c_begin(port)
            except Exception as exc:
                failures = True
                logger.warning(f"Failed to replay I2C begin on port {port}: {exc!r}")

        for pin, digital_mode in sorted(digital_modes.items()):
            try:
                bridge.set_digital_mode(pin, digital_mode)
            except Exception as exc:
                failures = True
                logger.warning(f"Failed to replay digital mode for pin {pin}: {exc!r}")

        for pin, analog_mode in sorted(analog_modes.items()):
            try:
                bridge.set_analog_mode(pin, analog_mode)
            except Exception as exc:
                failures = True
                logger.warning(f"Failed to replay analog mode for pin {pin}: {exc!r}")

        for port, address in sorted(ccs811_devices):
            try:
                bridge.i2c_ccs811_begin(port, address)
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay CCS811 begin on port "
                    f"{port} address {hex(address)}: {exc!r}"
                )

        for port, address in sorted(mpu6050_devices):
            try:
                bridge.i2c_mpu6050_begin(port, address)
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay MPU6050 begin on port "
                    f"{port} address {hex(address)}: {exc!r}"
                )

        logger.info("Finished replaying Telemetrix config")

        if failures:
            raise RuntimeError("Telemetrix config replay had failures")

    def dump(self) -> str:
        with self._lock:
            sampling_interval_ms: Optional[int] = self._sampling_interval_ms
            memory_reporting_interval_ms: Optional[int] = (
                self._memory_reporting_interval_ms
            )
            cpu_fan_sampling_interval_ms: Optional[int] = (
                self._cpu_fan_sampling_interval_ms
            )
            digital_modes_count: int = len(self._digital_modes)
            analog_modes_count: int = len(self._analog_modes)
            i2c_ports_count: int = len(self._i2c_ports_started)
            ccs811_devices_count: int = len(self._ccs811_devices)
            mpu6050_devices_count: int = len(self._mpu6050_devices)

        return (
            "intervals="
            f"mem:{memory_reporting_interval_ms},"
            f"sampling:{sampling_interval_ms},"
            f"cpu_fan:{cpu_fan_sampling_interval_ms} "
            "pins="
            f"digital:{digital_modes_count},"
            f"analog:{analog_modes_count} "
            "i2c="
            f"ports:{i2c_ports_count},"
            f"ccs811:{ccs811_devices_count},"
            f"mpu6050:{mpu6050_devices_count}"
        )
