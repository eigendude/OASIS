################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

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
    def info(self, msg: str) -> None:
        """Log an informational message"""

    def warning(self, msg: str) -> None:
        """Log a warning message"""


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

    def dump(self) -> str:
        with self._lock:
            sampling_interval_ms: Optional[int] = self._sampling_interval_ms
            memory_reporting_interval_ms: Optional[int] = (
                self._memory_reporting_interval_ms
            )
            cpu_fan_sampling_interval_ms: Optional[int] = (
                self._cpu_fan_sampling_interval_ms
            )
            digital_pin_count: int = len(self._digital_modes)
            analog_pin_count: int = len(self._analog_modes)
            i2c_port_count: int = len(self._i2c_ports_started)
            ccs811_count: int = len(self._ccs811_devices)
            mpu6050_count: int = len(self._mpu6050_devices)

        return self._format_summary(
            sampling_interval_ms,
            memory_reporting_interval_ms,
            cpu_fan_sampling_interval_ms,
            digital_pin_count,
            analog_pin_count,
            i2c_port_count,
            ccs811_count,
            mpu6050_count,
        )

    def replay(self, bridge: TelemetrixBridge, logger: TelemetrixLogger) -> None:
        with self._lock:
            sampling_interval_ms: Optional[int] = self._sampling_interval_ms
            memory_reporting_interval_ms: Optional[int] = (
                self._memory_reporting_interval_ms
            )
            cpu_fan_sampling_interval_ms: Optional[int] = (
                self._cpu_fan_sampling_interval_ms
            )
            digital_modes: Dict[int, DigitalMode] = dict(self._digital_modes)
            analog_modes: Dict[int, AnalogMode] = dict(self._analog_modes)
            i2c_ports_started: Set[int] = set(self._i2c_ports_started)
            ccs811_devices: Set[Tuple[int, int]] = set(self._ccs811_devices)
            mpu6050_devices: Set[Tuple[int, int]] = set(self._mpu6050_devices)

        logger.info(
            "Replaying Telemetrix config: "
            + self._format_summary(
                sampling_interval_ms,
                memory_reporting_interval_ms,
                cpu_fan_sampling_interval_ms,
                len(digital_modes),
                len(analog_modes),
                len(i2c_ports_started),
                len(ccs811_devices),
                len(mpu6050_devices),
            )
        )

        failures: int = 0

        if memory_reporting_interval_ms is not None:
            try:
                bridge.set_memory_reporting_interval(memory_reporting_interval_ms)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay memory reporting interval "
                    f"{memory_reporting_interval_ms} ms: {exc!r}"
                )

        if sampling_interval_ms is not None:
            try:
                bridge.set_sampling_interval(sampling_interval_ms)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay sampling interval "
                    f"{sampling_interval_ms} ms: {exc!r}"
                )

        if cpu_fan_sampling_interval_ms is not None:
            try:
                bridge.set_cpu_fan_sampling_interval(cpu_fan_sampling_interval_ms)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay CPU fan sampling interval "
                    f"{cpu_fan_sampling_interval_ms} ms: {exc!r}"
                )

        for i2c_port in sorted(i2c_ports_started):
            try:
                bridge.i2c_begin(i2c_port)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay I2C begin for port "
                    f"{i2c_port}: {exc!r}"
                )

        for pin, mode in sorted(digital_modes.items()):
            try:
                bridge.set_digital_mode(pin, mode)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay digital mode for pin "
                    f"{pin} ({mode}): {exc!r}"
                )

        for pin, mode in sorted(analog_modes.items()):
            try:
                bridge.set_analog_mode(pin, mode)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay analog mode for pin "
                    f"{pin} ({mode}): {exc!r}"
                )

        for i2c_port, i2c_address in sorted(ccs811_devices):
            try:
                bridge.i2c_ccs811_begin(i2c_port, i2c_address)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay CCS811 begin for port "
                    f"{i2c_port} address {hex(i2c_address)}: {exc!r}"
                )

        for i2c_port, i2c_address in sorted(mpu6050_devices):
            try:
                bridge.i2c_mpu6050_begin(i2c_port, i2c_address)
            except Exception as exc:
                failures += 1
                logger.warning(
                    "Failed to replay MPU6050 begin for port "
                    f"{i2c_port} address {hex(i2c_address)}: {exc!r}"
                )

        logger.info("Telemetrix config replay complete")

        if failures:
            raise RuntimeError("Telemetrix config replay had failures")

    @staticmethod
    def _format_summary(
        sampling_interval_ms: Optional[int],
        memory_reporting_interval_ms: Optional[int],
        cpu_fan_sampling_interval_ms: Optional[int],
        digital_pin_count: int,
        analog_pin_count: int,
        i2c_port_count: int,
        ccs811_count: int,
        mpu6050_count: int,
    ) -> str:
        sampling_summary: str = (
            str(sampling_interval_ms)
            if sampling_interval_ms is not None
            else "unset"
        )
        memory_summary: str = (
            str(memory_reporting_interval_ms)
            if memory_reporting_interval_ms is not None
            else "unset"
        )
        cpu_fan_summary: str = (
            str(cpu_fan_sampling_interval_ms)
            if cpu_fan_sampling_interval_ms is not None
            else "unset"
        )

        return (
            "intervals(ms)=(memory="
            + memory_summary
            + ", sampling="
            + sampling_summary
            + ", cpu_fan="
            + cpu_fan_summary
            + ") "
            + "digital_pins="
            + str(digital_pin_count)
            + " analog_pins="
            + str(analog_pin_count)
            + " i2c_ports="
            + str(i2c_port_count)
            + " ccs811_devices="
            + str(ccs811_count)
            + " mpu6050_devices="
            + str(mpu6050_count)
        )
