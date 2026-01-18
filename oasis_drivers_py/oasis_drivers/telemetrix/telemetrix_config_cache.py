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


class Logger(Protocol):
    def info(self, message: str) -> None: ...

    def warning(self, message: str) -> None: ...


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

    def replay(self, bridge: TelemetrixBridge, logger: Logger) -> None:
        with self._lock:
            memory_reporting_interval_ms = self._memory_reporting_interval_ms
            sampling_interval_ms = self._sampling_interval_ms
            cpu_fan_sampling_interval_ms = self._cpu_fan_sampling_interval_ms
            digital_modes = dict(self._digital_modes)
            analog_modes = dict(self._analog_modes)
            i2c_ports_started = set(self._i2c_ports_started)
            ccs811_devices = set(self._ccs811_devices)
            mpu6050_devices = set(self._mpu6050_devices)

        failures = 0

        logger.info(f"Replaying Telemetrix config: {self.summary()}")

        if memory_reporting_interval_ms is not None:
            try:
                bridge.set_memory_reporting_interval(memory_reporting_interval_ms)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay memory reporting interval "
                    f"{memory_reporting_interval_ms}: {exc!r}"
                )

        if sampling_interval_ms is not None:
            try:
                bridge.set_sampling_interval(sampling_interval_ms)
            except Exception as exc:
                failures += 1
                logger.warning(
                    f"Failed to replay sampling interval {sampling_interval_ms}: {exc!r}"
                )

        if cpu_fan_sampling_interval_ms is not None:
            try:
                bridge.set_cpu_fan_sampling_interval(cpu_fan_sampling_interval_ms)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay CPU fan sampling interval "
                    f"{cpu_fan_sampling_interval_ms}: {exc!r}"
                )

        for port in sorted(i2c_ports_started):
            try:
                bridge.i2c_begin(port)
            except Exception as exc:
                failures += 1
                logger.warning(f"Failed to replay I2C begin on port {port}: {exc!r}")

        for pin, mode in sorted(digital_modes.items()):
            try:
                bridge.set_digital_mode(pin, mode)
            except Exception as exc:
                failures += 1
                logger.warning(
                    f"Failed to replay digital mode for pin {pin} ({mode}): {exc!r}"
                )

        for pin, mode in sorted(analog_modes.items()):
            try:
                bridge.set_analog_mode(pin, mode)
            except Exception as exc:
                failures += 1
                logger.warning(
                    f"Failed to replay analog mode for pin {pin} ({mode}): {exc!r}"
                )

        for port, address in sorted(ccs811_devices):
            try:
                bridge.i2c_ccs811_begin(port, address)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay CCS811 begin on port "
                    f"{port} addr {hex(address)}: {exc!r}"
                )

        for port, address in sorted(mpu6050_devices):
            try:
                bridge.i2c_mpu6050_begin(port, address)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay MPU6050 begin on port "
                    f"{port} addr {hex(address)}: {exc!r}"
                )

        logger.info("Telemetrix config replay complete")

        if failures:
            raise RuntimeError("Telemetrix config replay had failures")

    def summary(self) -> str:
        with self._lock:
            sampling_interval_ms = self._sampling_interval_ms
            memory_reporting_interval_ms = self._memory_reporting_interval_ms
            cpu_fan_sampling_interval_ms = self._cpu_fan_sampling_interval_ms
            digital_count = len(self._digital_modes)
            analog_count = len(self._analog_modes)
            i2c_ports = len(self._i2c_ports_started)
            ccs811_count = len(self._ccs811_devices)
            mpu6050_count = len(self._mpu6050_devices)

        return (
            "intervals="
            f"(memory={memory_reporting_interval_ms}, "
            f"sampling={sampling_interval_ms}, "
            f"cpu_fan={cpu_fan_sampling_interval_ms}), "
            f"digital_pins={digital_count}, "
            f"analog_pins={analog_count}, "
            f"i2c_ports={i2c_ports}, "
            f"ccs811_devices={ccs811_count}, "
            f"mpu6050_devices={mpu6050_count}"
        )

    def __repr__(self) -> str:
        return f"TelemetrixConfigCache({self.summary()})"
