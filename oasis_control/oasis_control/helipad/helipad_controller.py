################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Core helipad landing guidance logic."""

from __future__ import annotations

import math


LAND_THRESHOLD_VOLTS: float = 4.6
LAND_DEBOUNCE_SECS: float = 0.2
GUIDANCE_PERIOD_SECS: float = 1.2
MAX_DUTY_CYCLE: float = 1.0


class HelipadController:
    """State machine for reflectance-based helipad guidance."""

    def __init__(
        self,
        land_threshold_volts: float = LAND_THRESHOLD_VOLTS,
        debounce_secs: float = LAND_DEBOUNCE_SECS,
        guidance_period_secs: float = GUIDANCE_PERIOD_SECS,
        max_duty_cycle: float = MAX_DUTY_CYCLE,
    ) -> None:
        if not math.isfinite(land_threshold_volts):
            raise ValueError("land_threshold_volts must be finite")
        if not math.isfinite(debounce_secs) or debounce_secs < 0.0:
            raise ValueError("debounce_secs must be finite and non-negative")
        if not math.isfinite(guidance_period_secs) or guidance_period_secs <= 0.0:
            raise ValueError("guidance_period_secs must be finite and positive")
        if not math.isfinite(max_duty_cycle) or not 0.0 < max_duty_cycle <= 1.0:
            raise ValueError("max_duty_cycle must be in the range (0.0, 1.0]")

        self._land_threshold_volts: float = land_threshold_volts
        self._debounce_secs: float = debounce_secs
        self._guidance_period_secs: float = guidance_period_secs
        self._max_duty_cycle: float = max_duty_cycle
        self._landed: bool = False
        self._candidate_landed: bool = False
        self._candidate_since_sec: float | None = None
        self._guidance_started_sec: float | None = None
        self._last_timestamp_sec: float | None = None

    @property
    def landed(self) -> bool:
        """Return whether the helicopter is currently considered landed."""

        return self._landed

    def update(
        self, timestamp_sec: float, sensor_voltage: float
    ) -> tuple[float, float]:
        """Advance the controller and return PWM duty cycles for both pairs."""

        current_timestamp_sec: float = self._normalize_timestamp(timestamp_sec)
        landing_changed: bool = self._update_landing_state(
            current_timestamp_sec,
            sensor_voltage,
        )

        if self._landed:
            return 0.0, 0.0

        if landing_changed:
            self._guidance_started_sec = current_timestamp_sec

        return self._compute_guidance_pwm(current_timestamp_sec)

    def _normalize_timestamp(self, timestamp_sec: float) -> float:
        if not math.isfinite(timestamp_sec):
            if self._last_timestamp_sec is not None:
                return self._last_timestamp_sec
            return 0.0

        current_timestamp_sec: float = timestamp_sec
        if self._last_timestamp_sec is not None:
            current_timestamp_sec = max(current_timestamp_sec, self._last_timestamp_sec)

        self._last_timestamp_sec = current_timestamp_sec
        return current_timestamp_sec

    def _update_landing_state(
        self, timestamp_sec: float, sensor_voltage: float
    ) -> bool:
        sensed_voltage: float = sensor_voltage if math.isfinite(sensor_voltage) else 0.0
        observed_landed: bool = sensed_voltage <= self._land_threshold_volts

        if observed_landed == self._landed:
            self._candidate_landed = observed_landed
            self._candidate_since_sec = None
            return False

        if (
            self._candidate_since_sec is None
            or self._candidate_landed != observed_landed
        ):
            self._candidate_landed = observed_landed
            self._candidate_since_sec = timestamp_sec
            return False

        if timestamp_sec - self._candidate_since_sec < self._debounce_secs:
            return False

        self._landed = observed_landed
        self._candidate_since_sec = None
        if not self._landed:
            self._guidance_started_sec = timestamp_sec

        return True

    def _compute_guidance_pwm(self, timestamp_sec: float) -> tuple[float, float]:
        if self._guidance_started_sec is None:
            self._guidance_started_sec = timestamp_sec

        elapsed_sec: float = timestamp_sec - self._guidance_started_sec
        cycle_phase: float = (elapsed_sec % self._guidance_period_secs) / (
            self._guidance_period_secs
        )

        if cycle_phase < 0.5:
            local_phase: float = cycle_phase / 0.5
            pair_a_duty: float = math.sin(math.pi * local_phase) * self._max_duty_cycle
            pair_b_duty: float = 0.0
        else:
            local_phase = (cycle_phase - 0.5) / 0.5
            pair_a_duty = 0.0
            pair_b_duty = math.sin(math.pi * local_phase) * self._max_duty_cycle

        return self._clamp_duty(pair_a_duty), self._clamp_duty(pair_b_duty)

    @staticmethod
    def _clamp_duty(duty_cycle: float) -> float:
        if not math.isfinite(duty_cycle):
            return 0.0
        return max(0.0, min(duty_cycle, 1.0))
