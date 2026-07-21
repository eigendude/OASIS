################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Checkerboard-triggered train park-mode state."""

from __future__ import annotations

from dataclasses import dataclass


# Seconds of tolerance for floating-point phase-boundary comparisons
PROFILE_TIME_EPSILON_SEC: float = 1.0e-12


@dataclass(frozen=True)
class ParkModeLaunchProfile:
    """Piecewise-linear reverse launch profile.

    Attributes:
        preload_command: Effective command at the end of fast preload. The
            expected range is (0.0, 1.0).
        preload_sec: Fast-preload duration in seconds. Must be finite and
            greater than zero.
        takeup_command: Effective command at the end of magnetic-coupler
            take-up. It must be greater than ``preload_command`` and less than
            ``command``.
        takeup_sec: Magnetic-coupler take-up duration in seconds. Must be
            finite and greater than zero.
        hold_sec: Duration in seconds to hold ``takeup_command`` before final
            acceleration. Must be finite and greater than or equal to zero.
        command: Final unboosted effective park command in (0.0, 1.0].
        accel_sec: Final-acceleration duration in seconds. Must be finite and
            greater than zero.
    """

    preload_command: float
    preload_sec: float
    takeup_command: float
    takeup_sec: float
    hold_sec: float
    command: float
    accel_sec: float


class TrainParkMode:
    """
    Tracks reverse park-mode state until a checkerboard interruption edge.

    Park mode drives the train backward while watching for the train end to
    interrupt the checkerboard. The first checkerboard message after activation
    initializes edge detection only, so activation while already interrupted
    does not immediately park the train.
    """

    def __init__(
        self,
        enabled: bool,
        profile: ParkModeLaunchProfile,
    ) -> None:
        """
        Initialize park-mode configuration.
        """
        self._enabled: bool = enabled
        self._profile: ParkModeLaunchProfile = profile
        self._active: bool = False
        self._ramp_start_sec: float | None = None
        self._last_update_sec: float | None = None
        self._last_target_command: float = 0.0
        self._applied_command: float = 0.0
        self._have_checkerboard_status: bool = False
        self._last_checkerboard_visible: bool = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def command(self) -> float:
        return self._profile.command

    @property
    def active(self) -> bool:
        return self._active

    def activate(self, now_sec: float) -> bool:
        if not self._enabled:
            return False

        self._active = True
        self._ramp_start_sec = now_sec
        self._last_update_sec = now_sec
        self._last_target_command = 0.0
        self._applied_command = 0.0
        self._have_checkerboard_status = False
        self._last_checkerboard_visible = False

        return True

    def cancel(self) -> bool:
        was_active: bool = self._active

        self._active = False
        self._ramp_start_sec = None
        self._last_update_sec = None
        self._last_target_command = 0.0
        self._applied_command = 0.0
        self._have_checkerboard_status = False
        self._last_checkerboard_visible = False

        return was_active

    def update_checkerboard_status(self, checkerboard_visible: bool) -> bool:
        if not self._active:
            return False

        if not self._have_checkerboard_status:
            self._have_checkerboard_status = True
            self._last_checkerboard_visible = checkerboard_visible
            return False

        parked: bool = self._last_checkerboard_visible and not checkerboard_visible
        self._last_checkerboard_visible = checkerboard_visible

        if parked:
            self._active = False
            self._ramp_start_sec = None
            self._last_update_sec = None
            self._last_target_command = 0.0
            self._applied_command = 0.0

        return parked

    def limit_command(self, target_command: float, now_sec: float) -> float:
        """Apply the launch profile and upward slew limit to a target."""
        if not self._active:
            return 0.0

        ramp_start_sec: float | None = self._ramp_start_sec
        last_update_sec: float | None = self._last_update_sec
        if (
            ramp_start_sec is None
            or last_update_sec is None
            or now_sec < last_update_sec
        ):
            self._restart_ramp(now_sec, target_command)
            return 0.0

        target_command = max(target_command, 0.0)
        elapsed_sec: float = now_sec - ramp_start_sec
        update_sec: float = now_sec - last_update_sec
        last_elapsed_sec: float = last_update_sec - ramp_start_sec
        profile_command: float = self._profile_command(elapsed_sec)
        requested_profile_command: float = min(target_command, profile_command)

        if target_command <= self._applied_command:
            limited_command: float = target_command
        else:
            limited_command = max(self._applied_command, requested_profile_command)

            target_increased: bool = target_command > self._last_target_command
            profile_end_sec: float = (
                self._profile.preload_sec
                + self._profile.takeup_sec
                + self._profile.hold_sec
                + self._profile.accel_sec
            )
            profile_was_complete: bool = (
                last_elapsed_sec + PROFILE_TIME_EPSILON_SEC >= profile_end_sec
            )
            if (
                target_command > self._profile.command
                and not target_increased
                and profile_was_complete
            ):
                # Effective command per second during final acceleration. The
                # same physical slew rate limits any command added by X boost
                accel_slew_rate: float = (
                    self._profile.command - self._profile.takeup_command
                ) / self._profile.accel_sec
                limited_command = max(
                    limited_command,
                    self._applied_command + update_sec * accel_slew_rate,
                )

            limited_command = min(limited_command, target_command)

        self._last_update_sec = now_sec
        self._last_target_command = target_command
        self._applied_command = limited_command

        return limited_command

    def _profile_command(self, elapsed_sec: float) -> float:
        if elapsed_sec <= 0.0:
            return 0.0

        profile: ParkModeLaunchProfile = self._profile
        if elapsed_sec < profile.preload_sec:
            return profile.preload_command * elapsed_sec / profile.preload_sec

        elapsed_sec -= profile.preload_sec
        if elapsed_sec < profile.takeup_sec:
            takeup_fraction: float = elapsed_sec / profile.takeup_sec
            return profile.preload_command + takeup_fraction * (
                profile.takeup_command - profile.preload_command
            )

        elapsed_sec -= profile.takeup_sec
        if elapsed_sec < profile.hold_sec:
            return profile.takeup_command

        elapsed_sec -= profile.hold_sec
        if elapsed_sec < profile.accel_sec:
            accel_fraction: float = elapsed_sec / profile.accel_sec
            return profile.takeup_command + accel_fraction * (
                profile.command - profile.takeup_command
            )

        return profile.command

    def _restart_ramp(self, now_sec: float, target_command: float) -> None:
        self._ramp_start_sec = now_sec
        self._last_update_sec = now_sec
        self._last_target_command = max(target_command, 0.0)
        self._applied_command = 0.0
