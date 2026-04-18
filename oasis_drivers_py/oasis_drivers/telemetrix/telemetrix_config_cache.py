################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
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
        self._helipad_config: Optional[Tuple[int, int, int]] = None
        self._helipad_mode: Optional[int] = None
        self._led_thruster_config: Optional[Tuple[int, int]] = None
        self._led_thruster_mode: Optional[int] = None

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

    def record_helipad_attach(
        self, ir_pin: int, led_pair_a_pin: int, led_pair_b_pin: int
    ) -> None:
        with self._lock:
            self._helipad_config = (ir_pin, led_pair_a_pin, led_pair_b_pin)

    def record_helipad_detach(self) -> None:
        with self._lock:
            self._helipad_config = None
            self._helipad_mode = None

    def record_helipad_mode(self, mode: int) -> None:
        with self._lock:
            self._helipad_mode = mode

    def record_led_thruster_attach(self, instance_id: int, led_pin: int) -> None:
        with self._lock:
            self._led_thruster_config = (instance_id, led_pin)

    def record_led_thruster_detach(self) -> None:
        with self._lock:
            self._led_thruster_config = None
            self._led_thruster_mode = None

    def record_led_thruster_state(self, mode: int) -> None:
        with self._lock:
            self._led_thruster_mode = mode

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
            helipad_config: Optional[Tuple[int, int, int]] = self._helipad_config
            helipad_mode: Optional[int] = self._helipad_mode
            led_thruster_config: Optional[Tuple[int, int]] = self._led_thruster_config
            led_thruster_mode: Optional[int] = self._led_thruster_mode

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

        if helipad_config is not None:
            ir_pin, led_pair_a_pin, led_pair_b_pin = helipad_config
            try:
                bridge.configure_effect(
                    effect_kind=1,
                    instance_id=0,
                    analog_pins=[ir_pin],
                    digital_pins=[],
                    pwm_pins=[led_pair_a_pin, led_pair_b_pin],
                    config_values=[],
                )
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay helipad attach on pins "
                    f"A{ir_pin}, D{led_pair_a_pin}, D{led_pair_b_pin}: {exc!r}"
                )

        if helipad_mode is not None:
            try:
                bridge.set_effect(
                    effect_kind=1,
                    instance_id=0,
                    mode=helipad_mode,
                    values=[],
                )
            except Exception as exc:
                failures = True
                logger.warning(f"Failed to replay helipad mode {helipad_mode}: {exc!r}")

        if led_thruster_config is not None:
            instance_id, led_pin = led_thruster_config
            try:
                bridge.configure_effect(
                    effect_kind=2,
                    instance_id=instance_id,
                    analog_pins=[],
                    digital_pins=[],
                    pwm_pins=[led_pin],
                    config_values=[],
                )
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay LED thruster attach on "
                    f"instance {instance_id} pin D{led_pin}: {exc!r}"
                )

        if led_thruster_mode is not None:
            instance_id = 0
            if led_thruster_config is not None:
                instance_id = led_thruster_config[0]

            try:
                bridge.set_effect(
                    effect_kind=2,
                    instance_id=instance_id,
                    mode=led_thruster_mode,
                    values=[],
                )
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay LED thruster mode "
                    f"{led_thruster_mode}: {exc!r}"
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
            helipad_attached: bool = self._helipad_config is not None
            helipad_mode: Optional[int] = self._helipad_mode
            led_thruster_attached: bool = self._led_thruster_config is not None
            led_thruster_mode: Optional[int] = self._led_thruster_mode

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
            f"mpu6050:{mpu6050_devices_count} "
            "helipad="
            f"attached:{helipad_attached},"
            f"mode:{helipad_mode} "
            "led_thruster="
            f"attached:{led_thruster_attached},"
            f"mode:{led_thruster_mode}"
        )
