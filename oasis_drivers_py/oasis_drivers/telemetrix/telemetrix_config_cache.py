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
from typing import Any
from typing import Callable
from typing import Dict
from typing import Optional
from typing import Protocol
from typing import Set
from typing import Tuple

from oasis_drivers.telemetrix.telemetrix_bridge import TelemetrixBridge
from oasis_drivers.telemetrix.telemetrix_config_store import TelemetrixConfigStore
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
    def __init__(self, store: TelemetrixConfigStore | None = None) -> None:
        self._lock = threading.Lock()
        self._store: TelemetrixConfigStore | None = store
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
        self._digital_output_values: Dict[int, bool] = {}
        self._pwm_output_values: Dict[int, float] = {}
        self._servo_output_values: Dict[int, float] = {}
        self._cpu_fan_output_values: Dict[int, float] = {}

    def record_sampling_interval(self, ms: int) -> None:
        self._update(lambda: self._record_sampling_interval(ms))

    def record_memory_reporting_interval(self, ms: int) -> None:
        self._update(lambda: self._record_memory_reporting_interval(ms))

    def record_cpu_fan_sampling_interval(self, ms: int) -> None:
        self._update(lambda: self._record_cpu_fan_sampling_interval(ms))

    def record_digital_mode(self, pin: int, mode: DigitalMode) -> None:
        self._update(lambda: self._record_digital_mode(pin, mode))

    def record_analog_mode(self, pin: int, mode: AnalogMode) -> None:
        self._update(lambda: self._record_analog_mode(pin, mode))

    def record_i2c_begin(self, port: int) -> None:
        self._update(lambda: self._record_i2c_begin(port))

    def record_ccs811_begin(self, port: int, address: int) -> None:
        self._update(lambda: self._record_ccs811_begin(port, address))

    def record_ccs811_end(self, port: int, address: int) -> None:
        self._update(lambda: self._record_ccs811_end(port, address))

    def record_mpu6050_begin(self, port: int, address: int) -> None:
        self._update(lambda: self._record_mpu6050_begin(port, address))

    def record_mpu6050_end(self, port: int, address: int) -> None:
        self._update(lambda: self._record_mpu6050_end(port, address))

    def record_helipad_attach(
        self, ir_pin: int, led_pair_a_pin: int, led_pair_b_pin: int
    ) -> None:
        self._update(
            lambda: self._record_helipad_attach(ir_pin, led_pair_a_pin, led_pair_b_pin)
        )

    def record_helipad_detach(self) -> None:
        self._update(self._record_helipad_detach)

    def record_helipad_mode(self, mode: int) -> None:
        self._update(lambda: self._record_helipad_mode(mode))

    def record_led_thruster_attach(self, instance_id: int, led_pin: int) -> None:
        self._update(lambda: self._record_led_thruster_attach(instance_id, led_pin))

    def record_led_thruster_detach(self) -> None:
        self._update(self._record_led_thruster_detach)

    def record_led_thruster_state(self, mode: int) -> None:
        self._update(lambda: self._record_led_thruster_state(mode))

    def record_digital_write(self, pin: int, value: bool) -> None:
        self._update(lambda: self._record_digital_write(pin, value))

    def record_pwm_write(self, pin: int, duty_cycle: float) -> None:
        self._update(lambda: self._record_pwm_write(pin, duty_cycle))

    def record_servo_write(self, pin: int, position: float) -> None:
        self._update(lambda: self._record_servo_write(pin, position))

    def record_cpu_fan_write(self, pin: int, duty_cycle: float) -> None:
        self._update(lambda: self._record_cpu_fan_write(pin, duty_cycle))

    def load(self, logger: TelemetrixLogger) -> None:
        store: TelemetrixConfigStore | None = self._store
        if store is None:
            return

        document: Optional[dict[str, Any]] = store.load()
        if document is None:
            logger.info(f"No Telemetrix cache found at {store.path}")
            return

        try:
            with self._lock:
                self._restore_locked(document)
        except (KeyError, TypeError, ValueError) as exc:
            logger.warning(
                f"Failed to load Telemetrix cache from {store.path}: {exc!r}"
            )
            return

        logger.info(f"Loaded Telemetrix cache from {store.path}: {self.dump()}")

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
            digital_output_values: Dict[int, bool] = dict(self._digital_output_values)
            pwm_output_values: Dict[int, float] = dict(self._pwm_output_values)
            servo_output_values: Dict[int, float] = dict(self._servo_output_values)
            cpu_fan_output_values: Dict[int, float] = dict(self._cpu_fan_output_values)

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

        for pin, value in sorted(digital_output_values.items()):
            try:
                bridge.digital_write(pin, value)
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay digital write for pin "
                    f"{pin} value {value}: {exc!r}"
                )

        for pin, duty_cycle in sorted(pwm_output_values.items()):
            try:
                bridge.pwm_write(pin, duty_cycle)
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay PWM write for pin "
                    f"{pin} duty cycle {duty_cycle}: {exc!r}"
                )

        for pin, position in sorted(servo_output_values.items()):
            try:
                bridge.servo_write(pin, position)
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay servo write for pin "
                    f"{pin} position {position}: {exc!r}"
                )

        for pin, duty_cycle in sorted(cpu_fan_output_values.items()):
            try:
                bridge.cpu_fan_write(pin, duty_cycle)
            except Exception as exc:
                failures = True
                logger.warning(
                    "Failed to replay CPU fan write for pin "
                    f"{pin} duty cycle {duty_cycle}: {exc!r}"
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
            digital_output_count: int = len(self._digital_output_values)
            pwm_output_count: int = len(self._pwm_output_values)
            servo_output_count: int = len(self._servo_output_values)
            cpu_fan_output_count: int = len(self._cpu_fan_output_values)

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
            f"mode:{led_thruster_mode} "
            "writes="
            f"digital:{digital_output_count},"
            f"pwm:{pwm_output_count},"
            f"servo:{servo_output_count},"
            f"cpu_fan:{cpu_fan_output_count}"
        )

    def _update(self, updater: Callable[[], None]) -> None:
        document: dict[str, object] | None = None

        with self._lock:
            updater()
            document = self._to_document_locked()

        self._save_document(document)

    def _save_document(self, document: dict[str, object]) -> None:
        store: TelemetrixConfigStore | None = self._store
        if store is None:
            return

        store.save(document)

    def _record_sampling_interval(self, ms: int) -> None:
        self._sampling_interval_ms = ms

    def _record_memory_reporting_interval(self, ms: int) -> None:
        self._memory_reporting_interval_ms = ms

    def _record_cpu_fan_sampling_interval(self, ms: int) -> None:
        self._cpu_fan_sampling_interval_ms = ms

    def _record_digital_mode(self, pin: int, mode: DigitalMode) -> None:
        self._digital_modes[pin] = mode

    def _record_analog_mode(self, pin: int, mode: AnalogMode) -> None:
        self._analog_modes[pin] = mode

    def _record_i2c_begin(self, port: int) -> None:
        self._i2c_ports_started.add(port)

    def _record_ccs811_begin(self, port: int, address: int) -> None:
        self._ccs811_devices.add((port, address))

    def _record_ccs811_end(self, port: int, address: int) -> None:
        self._ccs811_devices.discard((port, address))

    def _record_mpu6050_begin(self, port: int, address: int) -> None:
        self._mpu6050_devices.add((port, address))

    def _record_mpu6050_end(self, port: int, address: int) -> None:
        self._mpu6050_devices.discard((port, address))

    def _record_helipad_attach(
        self, ir_pin: int, led_pair_a_pin: int, led_pair_b_pin: int
    ) -> None:
        self._helipad_config = (ir_pin, led_pair_a_pin, led_pair_b_pin)

    def _record_helipad_detach(self) -> None:
        self._helipad_config = None
        self._helipad_mode = None

    def _record_helipad_mode(self, mode: int) -> None:
        self._helipad_mode = mode

    def _record_led_thruster_attach(self, instance_id: int, led_pin: int) -> None:
        self._led_thruster_config = (instance_id, led_pin)

    def _record_led_thruster_detach(self) -> None:
        self._led_thruster_config = None
        self._led_thruster_mode = None

    def _record_led_thruster_state(self, mode: int) -> None:
        self._led_thruster_mode = mode

    def _record_digital_write(self, pin: int, value: bool) -> None:
        self._digital_output_values[pin] = value

    def _record_pwm_write(self, pin: int, duty_cycle: float) -> None:
        self._pwm_output_values[pin] = duty_cycle

    def _record_servo_write(self, pin: int, position: float) -> None:
        self._servo_output_values[pin] = position

    def _record_cpu_fan_write(self, pin: int, duty_cycle: float) -> None:
        self._cpu_fan_output_values[pin] = duty_cycle

    def _to_document_locked(self) -> dict[str, object]:
        return {
            "schema_version": TelemetrixConfigStore.SCHEMA_VERSION,
            "sampling_interval_ms": self._sampling_interval_ms,
            "memory_reporting_interval_ms": self._memory_reporting_interval_ms,
            "cpu_fan_sampling_interval_ms": self._cpu_fan_sampling_interval_ms,
            "digital_modes": {
                str(pin): mode.name for pin, mode in sorted(self._digital_modes.items())
            },
            "analog_modes": {
                str(pin): mode.name for pin, mode in sorted(self._analog_modes.items())
            },
            "i2c_ports_started": sorted(self._i2c_ports_started),
            "ccs811_devices": [
                [port, address] for port, address in sorted(self._ccs811_devices)
            ],
            "mpu6050_devices": [
                [port, address] for port, address in sorted(self._mpu6050_devices)
            ],
            "helipad_config": (
                None if self._helipad_config is None else list(self._helipad_config)
            ),
            "helipad_mode": self._helipad_mode,
            "led_thruster_config": (
                None
                if self._led_thruster_config is None
                else list(self._led_thruster_config)
            ),
            "led_thruster_mode": self._led_thruster_mode,
            "digital_output_values": {
                str(pin): value
                for pin, value in sorted(self._digital_output_values.items())
            },
            "pwm_output_values": {
                str(pin): duty_cycle
                for pin, duty_cycle in sorted(self._pwm_output_values.items())
            },
            "servo_output_values": {
                str(pin): position
                for pin, position in sorted(self._servo_output_values.items())
            },
            "cpu_fan_output_values": {
                str(pin): duty_cycle
                for pin, duty_cycle in sorted(self._cpu_fan_output_values.items())
            },
        }

    def _restore_locked(self, document: dict[str, Any]) -> None:
        schema_version: int = int(document.get("schema_version", 0))
        if schema_version != TelemetrixConfigStore.SCHEMA_VERSION:
            raise ValueError(f"Unsupported schema version {schema_version}")

        self._sampling_interval_ms = self._optional_int(
            document.get("sampling_interval_ms")
        )
        self._memory_reporting_interval_ms = self._optional_int(
            document.get("memory_reporting_interval_ms")
        )
        self._cpu_fan_sampling_interval_ms = self._optional_int(
            document.get("cpu_fan_sampling_interval_ms")
        )
        self._digital_modes = self._parse_digital_modes(document.get("digital_modes"))
        self._analog_modes = self._parse_analog_modes(document.get("analog_modes"))
        self._i2c_ports_started = self._parse_int_set(
            document.get("i2c_ports_started"), field_name="i2c_ports_started"
        )
        self._ccs811_devices = self._parse_tuple_set(
            document.get("ccs811_devices"), field_name="ccs811_devices"
        )
        self._mpu6050_devices = self._parse_tuple_set(
            document.get("mpu6050_devices"), field_name="mpu6050_devices"
        )
        self._helipad_config = self._parse_optional_triplet(
            document.get("helipad_config"), field_name="helipad_config"
        )
        self._helipad_mode = self._optional_int(document.get("helipad_mode"))
        self._led_thruster_config = self._parse_optional_pair(
            document.get("led_thruster_config"), field_name="led_thruster_config"
        )
        self._led_thruster_mode = self._optional_int(document.get("led_thruster_mode"))
        self._digital_output_values = self._parse_bool_map(
            document.get("digital_output_values"),
            field_name="digital_output_values",
        )
        self._pwm_output_values = self._parse_float_map(
            document.get("pwm_output_values"),
            field_name="pwm_output_values",
        )
        self._servo_output_values = self._parse_float_map(
            document.get("servo_output_values"),
            field_name="servo_output_values",
        )
        self._cpu_fan_output_values = self._parse_float_map(
            document.get("cpu_fan_output_values"),
            field_name="cpu_fan_output_values",
        )

    def _parse_digital_modes(self, value: Any) -> Dict[int, DigitalMode]:
        raw_map: dict[str, Any] = self._require_dict(value, field_name="digital_modes")
        return {
            int(pin): DigitalMode[mode_name]
            for pin, mode_name in raw_map.items()
            if isinstance(mode_name, str)
        }

    def _parse_analog_modes(self, value: Any) -> Dict[int, AnalogMode]:
        raw_map: dict[str, Any] = self._require_dict(value, field_name="analog_modes")
        return {
            int(pin): AnalogMode[mode_name]
            for pin, mode_name in raw_map.items()
            if isinstance(mode_name, str)
        }

    def _parse_bool_map(self, value: Any, field_name: str) -> Dict[int, bool]:
        raw_map: dict[str, Any] = self._require_dict(value, field_name=field_name)
        return {int(pin): bool(pin_value) for pin, pin_value in raw_map.items()}

    def _parse_float_map(self, value: Any, field_name: str) -> Dict[int, float]:
        raw_map: dict[str, Any] = self._require_dict(value, field_name=field_name)
        return {int(pin): float(pin_value) for pin, pin_value in raw_map.items()}

    def _parse_int_set(self, value: Any, field_name: str) -> Set[int]:
        if value is None:
            return set()
        if not isinstance(value, list):
            raise TypeError(f"{field_name} must be a list")
        return {int(item) for item in value}

    def _parse_tuple_set(self, value: Any, field_name: str) -> Set[Tuple[int, int]]:
        if value is None:
            return set()
        if not isinstance(value, list):
            raise TypeError(f"{field_name} must be a list")

        result: Set[Tuple[int, int]] = set()
        item: Any
        for item in value:
            if not isinstance(item, list) or len(item) != 2:
                raise TypeError(f"{field_name} entries must be [port, address]")
            result.add((int(item[0]), int(item[1])))

        return result

    def _parse_optional_triplet(
        self, value: Any, field_name: str
    ) -> Optional[Tuple[int, int, int]]:
        if value is None:
            return None
        if not isinstance(value, list) or len(value) != 3:
            raise TypeError(f"{field_name} must be [a, b, c]")
        return int(value[0]), int(value[1]), int(value[2])

    def _parse_optional_pair(
        self, value: Any, field_name: str
    ) -> Optional[Tuple[int, int]]:
        if value is None:
            return None
        if not isinstance(value, list) or len(value) != 2:
            raise TypeError(f"{field_name} must be [a, b]")
        return int(value[0]), int(value[1])

    def _require_dict(self, value: Any, field_name: str) -> dict[str, Any]:
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise TypeError(f"{field_name} must be an object")
        return value

    def _optional_int(self, value: Any) -> Optional[int]:
        if value is None:
            return None
        return int(value)
