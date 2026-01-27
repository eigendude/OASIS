################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Mounting calibration pipeline orchestration."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np

from oasis_control.localization.mounting.config.mounting_config import MountingConfig
from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.config.mounting_params import SteadyParams
from oasis_control.localization.mounting.math_utils.quat import Quaternion
from oasis_control.localization.mounting.models.anchor_model import AnchorModel
from oasis_control.localization.mounting.models.diversity_metrics import (
    gravity_max_angle_deg,
)
from oasis_control.localization.mounting.models.diversity_metrics import (
    mag_proj_max_angle_deg,
)
from oasis_control.localization.mounting.models.imu_calibration_model import (
    ImuCalibrationModel,
)
from oasis_control.localization.mounting.models.imu_calibration_model import (
    ImuNuisanceState,
)
from oasis_control.localization.mounting.models.keyframe_clustering import (
    KeyframeClustering,
)
from oasis_control.localization.mounting.models.mag_direction_model import normalize
from oasis_control.localization.mounting.models.noise_adaptation import (
    DirectionNoiseAdapter,
)
from oasis_control.localization.mounting.models.steady_detector import SteadyDetector
from oasis_control.localization.mounting.models.steady_detector import (
    SteadyDetectorError,
)
from oasis_control.localization.mounting.mounting_types import Diagnostics
from oasis_control.localization.mounting.mounting_types import ImuPacket
from oasis_control.localization.mounting.mounting_types import Keyframe
from oasis_control.localization.mounting.mounting_types import MagPacket
from oasis_control.localization.mounting.mounting_types import ResultSnapshot
from oasis_control.localization.mounting.mounting_types import SteadySegment
from oasis_control.localization.mounting.mounting_types import UpdateReport
from oasis_control.localization.mounting.solver.initialization import (
    seed_keyframe_attitude_from_measurements,
)
from oasis_control.localization.mounting.solver.optimizer import optimize
from oasis_control.localization.mounting.state.mounting_state import ImuNuisance
from oasis_control.localization.mounting.state.mounting_state import KeyframeAttitude
from oasis_control.localization.mounting.state.mounting_state import MagNuisance
from oasis_control.localization.mounting.state.mounting_state import MountEstimate
from oasis_control.localization.mounting.state.mounting_state import MountingState
from oasis_control.localization.mounting.storage.persistence import save_yaml_snapshot
from oasis_control.localization.mounting.storage.yaml_format import DiversityYaml
from oasis_control.localization.mounting.storage.yaml_format import FlagsYaml
from oasis_control.localization.mounting.storage.yaml_format import FramesYaml
from oasis_control.localization.mounting.storage.yaml_format import ImuNuisanceYaml
from oasis_control.localization.mounting.storage.yaml_format import MagNuisanceYaml
from oasis_control.localization.mounting.storage.yaml_format import MountingSnapshotYaml
from oasis_control.localization.mounting.storage.yaml_format import MountRotationYaml
from oasis_control.localization.mounting.storage.yaml_format import QualityYaml
from oasis_control.localization.mounting.storage.yaml_format import snapshot_to_dict
from oasis_control.localization.mounting.tf.stability import RotationStabilityTracker
from oasis_control.localization.mounting.tf.stability import StabilityStatus
from oasis_control.localization.mounting.tf.tf_publisher import PublishedTransform
from oasis_control.localization.mounting.tf.tf_publisher import TfPublisher
from oasis_control.localization.mounting.timing.ring_buffer import TimestampRingBuffer
from oasis_control.localization.mounting.timing.time_base import TimeBase
from oasis_control.localization.mounting.timing.time_base import sec_to_ns


class MountingPipelineError(Exception):
    """Raised for mounting pipeline contract violations."""


@dataclass(frozen=True)
class PipelineOutputs:
    """Outputs produced by a mounting pipeline update step.

    Attributes:
        t_ns: Timestamp associated with the update
        snapshot: Current calibration snapshot when available
        report: Update report for the step
        published_transforms: TF publication outputs
        did_save: True when a snapshot was saved
        save_path: Path used when saving, if any
        diagnostics: Structured diagnostics bundle
    """

    t_ns: int
    snapshot: ResultSnapshot | None
    report: UpdateReport
    published_transforms: list[PublishedTransform]
    did_save: bool
    save_path: str | None
    diagnostics: object


class MountingPipeline:
    """End-to-end coordinator for AHRS mounting calibration."""

    def __init__(self, *, config: MountingConfig) -> None:
        """Initialize the mounting pipeline with configuration."""
        if not isinstance(config, MountingConfig):
            raise MountingPipelineError("config must be a MountingConfig")
        self._config: MountingConfig = config
        self._params: MountingParams = config.params
        self._time_base: TimeBase = TimeBase(allow_equal=True)

        self._steady_detector: SteadyDetector = SteadyDetector(self._params)
        self._keyframe_cluster: KeyframeClustering = KeyframeClustering(self._params)
        self._anchor: AnchorModel = AnchorModel()

        self._imu_nuisance: ImuNuisanceState = ImuCalibrationModel.default_state()
        self._mag_adapter: DirectionNoiseAdapter = DirectionNoiseAdapter(
            R_init=np.diag(self._params.mag.Rm_init_diag),
            R_min=np.diag(self._params.mag.Rm_min_diag),
            R_max=np.diag(self._params.mag.Rm_max_diag),
            alpha=float(self._params.mag.Rm_alpha),
        )

        self._stability_tracker: RotationStabilityTracker | None = None
        if (
            self._params.stability.stable_window_sec is not None
            and self._params.stability.stable_rot_thresh_rad is not None
        ):
            self._stability_tracker = RotationStabilityTracker(
                stable_window_sec=float(self._params.stability.stable_window_sec),
                stable_rot_thresh_rad=float(
                    self._params.stability.stable_rot_thresh_rad
                ),
                require_need_more_tilt_false=(
                    self._params.stability.require_need_more_tilt_false
                ),
                require_need_more_yaw_false=(
                    self._params.stability.require_need_more_yaw_false
                ),
            )

        self._tf_publisher: TfPublisher | None = None

        self._pending_segments: list[SteadySegment] = []
        self._mag_buffer: TimestampRingBuffer[MagPacket] = TimestampRingBuffer(
            capacity=256
        )
        self._last_mag_used_ns: int | None = None

        self._initialized: bool = False
        self._bootstrapping: bool = True
        self._bootstrap_start_ns: int | None = None
        self._bootstrap_omega: list[np.ndarray] = []
        self._bootstrap_accel: list[np.ndarray] = []
        self._bootstrap_mag: list[np.ndarray] = []
        self._bootstrap_stats: dict[str, np.ndarray] = {}

        self._state: MountingState = MountingState.default()
        self._imu_frame_id: str | None = None
        self._mag_frame_id: str | None = None

        self._last_imu_time_ns: int | None = None
        self._last_mag_time_ns: int | None = None
        self._last_step_time_ns: int | None = None
        self._last_solver_update_ns: int | None = None
        self._last_save_time_ns: int | None = None
        self._last_update_metrics: dict[str, Any] = {}

        self._raw_sample_count: int = 0
        self._segment_count: int = 0
        self._dropped_packets: int = 0
        self._save_fail_count: int = 0
        self._last_stability: StabilityStatus | None = None

    def reset(self) -> None:
        """Reset internal state to a clean startup condition."""
        self._steady_detector.reset()
        self._keyframe_cluster.reset()
        self._anchor.reset()
        self._mag_adapter = DirectionNoiseAdapter(
            R_init=np.diag(self._params.mag.Rm_init_diag),
            R_min=np.diag(self._params.mag.Rm_min_diag),
            R_max=np.diag(self._params.mag.Rm_max_diag),
            alpha=float(self._params.mag.Rm_alpha),
        )
        if self._stability_tracker is not None:
            self._stability_tracker.reset()
        self._tf_publisher = None

        self._pending_segments = []
        self._mag_buffer.clear()
        self._last_mag_used_ns = None

        self._initialized = False
        self._bootstrapping = True
        self._bootstrap_start_ns = None
        self._bootstrap_omega = []
        self._bootstrap_accel = []
        self._bootstrap_mag = []
        self._bootstrap_stats = {}
        self._imu_nuisance = ImuCalibrationModel.default_state()

        self._state = MountingState.default()
        self._imu_frame_id = None
        self._mag_frame_id = None

        self._last_imu_time_ns = None
        self._last_mag_time_ns = None
        self._last_step_time_ns = None
        self._last_solver_update_ns = None
        self._last_save_time_ns = None
        self._last_update_metrics = {}

        self._raw_sample_count = 0
        self._segment_count = 0
        self._dropped_packets = 0
        self._save_fail_count = 0
        self._last_stability = None

    def ingest_imu_pair(self, *, imu_packet: ImuPacket) -> PipelineOutputs | None:
        """Ingest a paired IMU packet with calibration prior."""
        if not isinstance(imu_packet, ImuPacket):
            raise MountingPipelineError("imu_packet must be an ImuPacket")
        if imu_packet.t_meas_ns < 0:
            self._drop_packet("imu_packet.t_meas_ns must be non-negative")
            return None
        if not imu_packet.frame_id:
            self._drop_packet("imu_packet.frame_id must be non-empty")
            return None
        if imu_packet.calibration is None:
            self._drop_packet("imu_packet.calibration must be provided")
            return None
        try:
            self._time_base.validate_non_decreasing(
                self._last_imu_time_ns,
                imu_packet.t_meas_ns,
            )
        except Exception as exc:
            raise MountingPipelineError(str(exc)) from exc

        self._last_imu_time_ns = imu_packet.t_meas_ns
        self._imu_frame_id = imu_packet.frame_id
        self._raw_sample_count += 1

        if imu_packet.calibration.valid and not self._initialized:
            self._imu_nuisance = ImuCalibrationModel.state_from_prior(
                imu_packet.calibration
            )

        omega_corr: np.ndarray = self._imu_nuisance.correct_gyro(
            imu_packet.omega_raw_rads
        )
        accel_corr: np.ndarray = self._imu_nuisance.correct_accel(imu_packet.a_raw_mps2)

        if self._bootstrapping:
            self._update_bootstrap_stats(
                omega_corr=omega_corr,
                accel_corr=accel_corr,
                t_ns=imu_packet.t_meas_ns,
            )

        mag_packet: MagPacket | None = self._consume_mag_for_steady(
            imu_packet.t_meas_ns
        )
        try:
            segment: SteadySegment | None = self._steady_detector.push(
                t_ns=imu_packet.t_meas_ns,
                omega_corr_rads=omega_corr,
                a_corr_mps2=accel_corr,
                imu_frame_id=imu_packet.frame_id,
                mag=mag_packet,
            )
        except SteadyDetectorError as exc:
            self._drop_packet(str(exc))
            return None

        if segment is not None:
            self._pending_segments.append(segment)

        return None

    def ingest_mag(self, *, mag_packet: MagPacket) -> None:
        """Ingest a magnetometer packet for later steady detection."""
        if not isinstance(mag_packet, MagPacket):
            raise MountingPipelineError("mag_packet must be a MagPacket")
        if mag_packet.t_meas_ns < 0:
            self._drop_packet("mag_packet.t_meas_ns must be non-negative")
            return
        if not mag_packet.frame_id:
            self._drop_packet("mag_packet.frame_id must be non-empty")
            return
        try:
            self._time_base.validate_non_decreasing(
                self._last_mag_time_ns,
                mag_packet.t_meas_ns,
            )
        except Exception as exc:
            raise MountingPipelineError(str(exc)) from exc

        self._mag_frame_id = mag_packet.frame_id
        self._last_mag_time_ns = mag_packet.t_meas_ns
        self._mag_buffer.push(mag_packet.t_meas_ns, mag_packet)

        if self._bootstrapping:
            self._update_bootstrap_mag(mag_packet=mag_packet)

    def step(self, *, t_now_ns: int) -> PipelineOutputs:
        """Advance the pipeline one update tick."""
        if not isinstance(t_now_ns, int) or isinstance(t_now_ns, bool):
            raise MountingPipelineError("t_now_ns must be an int")
        if t_now_ns < 0:
            raise MountingPipelineError("t_now_ns must be non-negative")
        if self._last_step_time_ns is not None and t_now_ns < self._last_step_time_ns:
            raise MountingPipelineError("t_now_ns must be non-decreasing")
        self._last_step_time_ns = t_now_ns

        self._maybe_complete_bootstrap(t_now_ns)
        dropped_segments: int = 0
        processed_segments: bool = False
        if self._pending_segments:
            dropped_segments = self._process_pending_segments()
            processed_segments = True

        need_more_tilt: bool
        need_more_yaw: bool
        diversity_tilt_deg: float | None
        diversity_yaw_deg: float | None
        (
            need_more_tilt,
            need_more_yaw,
            diversity_tilt_deg,
            diversity_yaw_deg,
        ) = self._compute_diversity_flags()

        did_update: bool = False
        update_metrics: dict[str, Any] = {}
        if self._should_run_solver(t_now_ns, processed_segments):
            did_update = self._run_solver()
            update_metrics = dict(self._last_update_metrics)

        is_stable: bool = False
        if self._stability_tracker is not None and self._initialized:
            stability: StabilityStatus = self._stability_tracker.update(
                t_ns=t_now_ns,
                R_BI=self._rotation_matrix(self._state.mount.q_BI_wxyz),
                R_BM=self._rotation_matrix(self._state.mount.q_BM_wxyz),
                need_more_tilt=need_more_tilt,
                need_more_yaw=need_more_yaw,
                mag_used_for_yaw=self._mag_used_for_yaw(),
            )
            self._last_stability = stability
            is_stable = stability.is_stable

        snapshot: ResultSnapshot | None = None
        if self._initialized and self._imu_frame_id is not None:
            snapshot = self._build_snapshot(
                t_ns=t_now_ns,
                diversity_tilt_deg=diversity_tilt_deg,
                diversity_yaw_deg=diversity_yaw_deg,
                is_stable=is_stable,
            )

        did_save: bool = False
        save_path: str | None = None
        if self._initialized:
            did_save, save_path = self._maybe_save_snapshot(
                t_ns=t_now_ns,
                diversity_tilt_deg=diversity_tilt_deg,
                diversity_yaw_deg=diversity_yaw_deg,
            )

        published_transforms: list[PublishedTransform] = []
        if self._initialized:
            self._ensure_tf_publisher()
        if self._initialized and self._tf_publisher is not None:
            R_BI: np.ndarray = self._rotation_from_state(self._state.mount.q_BI_wxyz)
            R_BM: np.ndarray = self._rotation_from_state(self._state.mount.q_BM_wxyz)
            published_transforms = self._tf_publisher.update(
                t_ns=t_now_ns,
                R_BI=R_BI,
                R_BM=R_BM,
                is_stable=is_stable,
                saved=did_save,
            )

        report: UpdateReport = self._build_update_report(
            did_update=did_update,
            update_metrics=update_metrics,
            need_more_tilt=need_more_tilt,
            need_more_yaw=need_more_yaw,
            dropped_segments=dropped_segments,
            dropped_mags=0,
        )

        diagnostics: Diagnostics = self._build_diagnostics(
            t_ns=t_now_ns,
            report=report,
            diversity_tilt_deg=diversity_tilt_deg,
            diversity_yaw_deg=diversity_yaw_deg,
            did_save=did_save,
        )

        return PipelineOutputs(
            t_ns=t_now_ns,
            snapshot=snapshot,
            report=report,
            published_transforms=published_transforms,
            did_save=did_save,
            save_path=save_path,
            diagnostics=diagnostics,
        )

    def is_bootstrapping(self) -> bool:
        """Return True while the pipeline is in bootstrap."""
        return self._bootstrapping

    def is_initialized(self) -> bool:
        """Return True when initialization has completed."""
        return self._initialized

    def steady_window_is_steady(self) -> bool:
        """Return True when the steady detector window is steady."""
        return self._steady_detector.window_is_steady()

    def gravity_direction_W_unit(self) -> np.ndarray | None:
        """Return the current gravity direction estimate in the world frame."""
        if not self._initialized:
            return None
        return np.array(self._state.g_W_unit, dtype=np.float64)

    def current_flags(self) -> FlagsYaml | None:
        """Return the current snapshot flags when initialized."""
        if not self._initialized:
            return None
        return FlagsYaml(
            anchored=self._state.anchored,
            mag_reference_invalid=self._state.mag_reference_invalid,
            mag_disturbance_detected=False,
            mag_dir_prior_from_driver_cov=self._params.mag.use_driver_cov_as_prior,
        )

    def keyframe_count(self) -> int:
        """Return the current number of keyframes."""
        return len(self._keyframe_cluster.keyframes())

    def raw_sample_count(self) -> int:
        """Return the total number of IMU samples ingested."""
        return self._raw_sample_count

    def segment_count(self) -> int:
        """Return the number of steady segments processed."""
        return self._segment_count

    def last_save_time_ns(self) -> int | None:
        """Return the last save timestamp in nanoseconds."""
        return self._last_save_time_ns

    def save_fail_count(self) -> int:
        """Return the total number of snapshot save failures."""
        return self._save_fail_count

    def bootstrap_remaining_sec(self, t_now_ns: int) -> float:
        """Return remaining bootstrap time in seconds."""
        if not self._bootstrapping or self._bootstrap_start_ns is None:
            return 0.0
        elapsed_ns: int = max(0, t_now_ns - self._bootstrap_start_ns)
        remaining_ns: int = max(
            0,
            sec_to_ns(self._params.bootstrap.bootstrap_sec) - elapsed_ns,
        )
        return float(remaining_ns) / 1e9

    def _drop_packet(self, message: str) -> None:
        """Record a dropped packet diagnostic."""
        self._dropped_packets += 1

    def _update_bootstrap_stats(
        self,
        *,
        omega_corr: np.ndarray,
        accel_corr: np.ndarray,
        t_ns: int,
    ) -> None:
        """Update bootstrap statistics with corrected IMU samples."""
        if self._bootstrap_start_ns is None:
            self._bootstrap_start_ns = t_ns
        self._bootstrap_omega.append(np.array(omega_corr, dtype=np.float64))
        self._bootstrap_accel.append(np.array(accel_corr, dtype=np.float64))

    def _update_bootstrap_mag(self, *, mag_packet: MagPacket) -> None:
        """Store bootstrap magnetometer samples when available."""
        self._bootstrap_mag.append(np.array(mag_packet.m_raw_T, dtype=np.float64))

    def _consume_mag_for_steady(self, t_ns: int) -> MagPacket | None:
        """Return the newest magnetometer sample for steady detection."""
        latest: MagPacket | None = None
        latest_t_ns: int | None = None
        for item in self._mag_buffer.iter_time_order():
            mag_t_ns: int
            mag: MagPacket
            if isinstance(item, tuple):
                mag_t_ns_raw, mag = item
                mag_t_ns = int(mag_t_ns_raw)
            else:
                mag_t_ns = int(item.t_ns)
                mag = item.value
            if mag_t_ns > t_ns:
                continue
            if (
                self._last_mag_used_ns is not None
                and mag_t_ns <= self._last_mag_used_ns
            ):
                continue
            latest = mag
            latest_t_ns = mag_t_ns
        if latest is None:
            return None
        self._last_mag_used_ns = latest.t_meas_ns
        if latest_t_ns is not None:
            self._mag_buffer.pop_older_than(latest_t_ns + 1)
        return latest

    def _maybe_complete_bootstrap(self, t_now_ns: int) -> None:
        """Complete bootstrap when duration has elapsed."""
        if not self._bootstrapping:
            return
        if self._bootstrap_start_ns is None:
            return
        bootstrap_ns: int = sec_to_ns(self._params.bootstrap.bootstrap_sec)
        if t_now_ns - self._bootstrap_start_ns < bootstrap_ns:
            return

        self._bootstrap_stats = self._compute_bootstrap_stats()
        self._maybe_update_steady_thresholds()

        g_ref: np.ndarray = self._bootstrap_gravity_ref()
        m_ref: np.ndarray | None = self._bootstrap_mag_ref()

        self._anchor.reset()
        self._anchor.capture(g_ref_W=g_ref, m_ref_W=m_ref)

        self._initialize_state()
        self._bootstrapping = False

    def _bootstrap_gravity_ref(self) -> np.ndarray:
        """Return the bootstrap gravity reference direction."""
        if not self._bootstrap_accel:
            return np.array([0.0, 0.0, -1.0], dtype=np.float64)
        accel_stack: np.ndarray = np.stack(self._bootstrap_accel, axis=0)
        mean_accel: np.ndarray = np.mean(accel_stack, axis=0)
        gravity_dir: np.ndarray = -mean_accel
        norm: float = float(np.linalg.norm(gravity_dir))
        if not np.isfinite(norm) or norm <= 0.0:
            return np.array([0.0, 0.0, -1.0], dtype=np.float64)
        return gravity_dir / norm

    def _bootstrap_mag_ref(self) -> np.ndarray | None:
        """Return the bootstrap magnetometer reference direction."""
        if not self._bootstrap_mag:
            return None
        mag_stack: np.ndarray = np.stack(self._bootstrap_mag, axis=0)
        mean_mag: np.ndarray = np.mean(mag_stack, axis=0)
        try:
            return normalize(mean_mag)
        except Exception:
            return None

    def _compute_bootstrap_stats(self) -> dict[str, np.ndarray]:
        """Compute stationary statistics from bootstrap samples."""
        stats: dict[str, np.ndarray] = {}
        if self._bootstrap_omega:
            omega_stack: np.ndarray = np.stack(self._bootstrap_omega, axis=0)
            omega_mean: np.ndarray = np.mean(omega_stack, axis=0)
            omega_centered: np.ndarray = omega_stack - omega_mean
            omega_cov: np.ndarray = (
                omega_centered.T @ omega_centered / float(len(self._bootstrap_omega))
            )
            stats["omega_mean"] = omega_mean
            stats["omega_cov"] = 0.5 * (omega_cov + omega_cov.T)
        if self._bootstrap_accel:
            accel_stack: np.ndarray = np.stack(self._bootstrap_accel, axis=0)
            accel_mean: np.ndarray = np.mean(accel_stack, axis=0)
            accel_centered: np.ndarray = accel_stack - accel_mean
            accel_cov: np.ndarray = (
                accel_centered.T @ accel_centered / float(len(self._bootstrap_accel))
            )
            stats["accel_mean"] = accel_mean
            stats["accel_cov"] = 0.5 * (accel_cov + accel_cov.T)
        return stats

    def _maybe_update_steady_thresholds(self) -> None:
        """Update steady thresholds using bootstrap statistics when configured."""
        if self._params.steady.omega_cov_thresh is not None:
            return
        if self._params.steady.k_omega is None:
            return
        omega_cov: np.ndarray | None = self._bootstrap_stats.get("omega_cov")
        if omega_cov is None:
            return
        omega_thresh: float = float(self._params.steady.k_omega) * float(
            np.trace(omega_cov)
        )
        steady_params: SteadyParams = replace(
            self._params.steady,
            omega_cov_thresh=omega_thresh,
        )
        self._params = self._params.replace(steady=steady_params)
        self._steady_detector = SteadyDetector(self._params)

    def _initialize_state(self) -> None:
        """Initialize mounting state after bootstrap."""
        imu_nuisance: ImuNuisance = ImuNuisance(
            b_a_mps2=self._imu_nuisance.b_a_mps2,
            A_a=self._imu_nuisance.A_a,
            b_g_rads=self._imu_nuisance.b_g_rads,
        )
        mag_nuisance: MagNuisance = MagNuisance(
            b_m_T=None,
            R_m_unitless2=self._mag_adapter.R(),
        )
        mount: MountEstimate = MountEstimate(
            q_BI_wxyz=self._state.mount.q_BI_wxyz,
            q_BM_wxyz=self._state.mount.q_BM_wxyz,
        )
        g_ref: np.ndarray = self._anchor.gravity_ref_W()
        m_ref: np.ndarray | None
        if self._anchor.has_mag_reference():
            m_ref = self._anchor.mag_ref_W()
        else:
            m_ref = None
        mag_reference_invalid: bool = not self._anchor.has_mag_reference()
        anchored: bool = True
        if self._params.bootstrap.mag_reference_required and mag_reference_invalid:
            anchored = False
        self._state = MountingState(
            mount=mount,
            g_W_unit=g_ref,
            m_W_unit=m_ref,
            imu=imu_nuisance,
            mag=mag_nuisance,
            keyframes=(),
            anchored=anchored,
            mag_reference_invalid=mag_reference_invalid,
        )
        # Seed keyframe attitudes for segments captured during bootstrap
        keyframes: tuple[Keyframe, ...] = self._keyframe_cluster.keyframes()
        if keyframes:
            attitudes: list[KeyframeAttitude] = []
            keyframe: Keyframe
            for keyframe in keyframes:
                q_wb: np.ndarray = seed_keyframe_attitude_from_measurements(
                    keyframe.gravity_unit_mean_dir_I(),
                    keyframe.mag_mean_dir_M,
                    self._state.mount.q_BI_wxyz,
                    self._state.mount.q_BM_wxyz,
                )
                attitudes.append(
                    KeyframeAttitude(
                        keyframe_id=keyframe.keyframe_id,
                        q_WB_wxyz=q_wb,
                    )
                )
            self._state = self._state.replace(keyframes=tuple(attitudes))
        self._initialized = True

    def _ensure_tf_publisher(self) -> None:
        """Create the TF publisher once frame identifiers are known."""
        if self._tf_publisher is None:
            if self._imu_frame_id is None or not self._imu_frame_id:
                return
            self._tf_publisher = TfPublisher(
                base_frame=self._params.frames.base_frame,
                imu_frame=self._imu_frame_id,
                mag_frame=self._mag_frame_id,
                publish_dynamic=self._params.tf.publish_dynamic,
                publish_static_when_stable=self._params.tf.publish_static_when_stable,
                republish_static_on_save=self._params.tf.republish_static_on_save,
            )
            return
        if self._mag_frame_id:
            self._tf_publisher.set_mag_frame(self._mag_frame_id)

    def _process_pending_segments(self) -> int:
        """Process steady segments into keyframes and state."""
        prior_keyframes: tuple[Keyframe, ...] = self._keyframe_cluster.keyframes()
        prior_ids: set[int] = {frame.keyframe_id for frame in prior_keyframes}
        for segment in self._pending_segments:
            keyframe_id: int
            keyframe: Keyframe
            keyframe_id, keyframe = self._keyframe_cluster.add_segment(segment)
            self._segment_count += 1
            if self._imu_frame_id is None:
                self._imu_frame_id = segment.imu_frame_id
            if segment.mag_frame_id is not None:
                self._mag_frame_id = segment.mag_frame_id

            if keyframe_id not in self._state.keyframe_ids():
                q_wb: np.ndarray = seed_keyframe_attitude_from_measurements(
                    keyframe.gravity_unit_mean_dir_I(),
                    keyframe.mag_mean_dir_M,
                    self._state.mount.q_BI_wxyz,
                    self._state.mount.q_BM_wxyz,
                )
                attitude: KeyframeAttitude = KeyframeAttitude(
                    keyframe_id=keyframe_id,
                    q_WB_wxyz=q_wb,
                )
                self._state = self._state.with_updated_keyframe(attitude)

        self._pending_segments = []
        current_keyframes: tuple[Keyframe, ...] = self._keyframe_cluster.keyframes()
        current_ids: set[int] = {frame.keyframe_id for frame in current_keyframes}
        dropped_ids: set[int] = prior_ids.difference(current_ids)

        if dropped_ids:
            remaining_attitudes: tuple[KeyframeAttitude, ...] = tuple(
                attitude
                for attitude in self._state.keyframes
                if attitude.keyframe_id in current_ids
            )
            self._state = self._state.replace(keyframes=remaining_attitudes)

        return len(dropped_ids)

    def _compute_diversity_flags(
        self,
    ) -> tuple[bool, bool, float | None, float | None]:
        """Return diversity flags and angle metrics."""
        keyframes: tuple[Keyframe, ...] = self._keyframe_cluster.keyframes()
        tilt_deg: float | None = gravity_max_angle_deg(list(keyframes))
        yaw_deg: float | None = mag_proj_max_angle_deg(list(keyframes))

        need_more_tilt: bool = False
        need_more_yaw: bool = False

        if self._params.diversity.N_min is not None:
            if self._segment_count < int(self._params.diversity.N_min):
                need_more_tilt = True
                if self._mag_used_for_yaw():
                    need_more_yaw = True

        if self._params.diversity.tilt_min_deg is not None:
            if tilt_deg is None or tilt_deg < float(
                self._params.diversity.tilt_min_deg
            ):
                need_more_tilt = True

        if self._mag_used_for_yaw():
            if self._params.diversity.yaw_min_deg is not None:
                if yaw_deg is None or yaw_deg < float(
                    self._params.diversity.yaw_min_deg
                ):
                    need_more_yaw = True
        else:
            need_more_yaw = False

        return need_more_tilt, need_more_yaw, tilt_deg, yaw_deg

    def _mag_used_for_yaw(self) -> bool:
        """Return True when mag diversity should be enforced."""
        if not self._params.diversity.use_mag_for_yaw:
            return False
        keyframes: tuple[Keyframe, ...] = self._keyframe_cluster.keyframes()
        return any(frame.mag_weight > 0 for frame in keyframes)

    def _should_run_solver(self, t_ns: int, processed_segments: bool) -> bool:
        """Return True when the solver should update."""
        if not self._initialized:
            return False
        if not self._keyframe_cluster.keyframes():
            return False
        if processed_segments:
            return True
        if self._params.solver.update_rate_hz is None:
            return False
        period_ns: int = sec_to_ns(1.0 / float(self._params.solver.update_rate_hz))
        if self._last_solver_update_ns is None:
            return True
        return (t_ns - self._last_solver_update_ns) >= period_ns

    def _run_solver(self) -> bool:
        """Run the optimizer and update state."""
        keyframes: tuple[Keyframe, ...] = self._keyframe_cluster.keyframes()
        if not keyframes:
            return False
        updated_state: MountingState
        metrics: dict[str, Any]
        updated_state, metrics = optimize(
            self._state,
            keyframes,
            self._params,
        )
        self._state = updated_state
        self._last_update_metrics = metrics
        if self._last_step_time_ns is not None:
            self._last_solver_update_ns = self._last_step_time_ns
        return True

    def _rotation_matrix(self, q_wxyz: np.ndarray) -> np.ndarray:
        """Return a rotation matrix from a quaternion."""
        quat: Quaternion = Quaternion(q_wxyz).normalized()
        return quat.as_matrix()

    def _rotation_from_state(self, q_wxyz: np.ndarray) -> np.ndarray:
        """Return a rotation matrix from a quaternion."""
        quat: Quaternion = Quaternion(q_wxyz).normalized()
        return quat.as_matrix()

    def _build_snapshot(
        self,
        *,
        t_ns: int,
        diversity_tilt_deg: float | None,
        diversity_yaw_deg: float | None,
        is_stable: bool,
    ) -> ResultSnapshot:
        """Build a ResultSnapshot from the current state."""
        R_BI: np.ndarray = self._rotation_from_state(
            self._state.mount.q_BI_wxyz,
        )
        R_BM: np.ndarray | None = None
        frame_mag: str | None = None
        if self._mag_frame_id is not None:
            R_BM = self._rotation_from_state(
                self._state.mount.q_BM_wxyz,
            )
            frame_mag = self._mag_frame_id

        b_m_T: np.ndarray | None
        R_m: np.ndarray | None
        if frame_mag is None:
            b_m_T = None
            R_m = None
        else:
            b_m_T = self._state.mag.b_m_T
            R_m = self._state.mag.R_m_unitless2

        cov_zero: np.ndarray = np.zeros((3, 3), dtype=np.float64)
        return ResultSnapshot(
            t_meas_ns=t_ns,
            frame_base=self._params.frames.base_frame,
            frame_imu=self._imu_frame_id or "",
            frame_mag=frame_mag,
            R_BI=R_BI,
            R_BM=R_BM,
            cov_rot_BI=cov_zero,
            cov_rot_BM=cov_zero if frame_mag is not None else None,
            b_a_mps2=self._state.imu.b_a_mps2,
            A_a=self._state.imu.A_a,
            b_g_rads=self._state.imu.b_g_rads,
            b_m_T=b_m_T,
            R_m=R_m,
            segment_count=self._segment_count,
            keyframe_count=len(self._keyframe_cluster.keyframes()),
            diversity_tilt_deg=diversity_tilt_deg,
            diversity_yaw_deg=diversity_yaw_deg,
            is_stable=is_stable,
            quality_score=None,
        )

    def _maybe_save_snapshot(
        self,
        *,
        t_ns: int,
        diversity_tilt_deg: float | None,
        diversity_yaw_deg: float | None,
    ) -> tuple[bool, str | None]:
        """Persist a calibration snapshot when the save policy triggers."""
        output_path: str | None = self._params.save.output_path
        if output_path is None or output_path == "":
            return False, None
        period_ns: int = sec_to_ns(self._params.save.save_period_sec)
        if self._last_save_time_ns is not None:
            if t_ns - self._last_save_time_ns < period_ns:
                return False, None

        snapshot: MountingSnapshotYaml = self._build_snapshot_yaml(
            t_ns=t_ns,
            diversity_tilt_deg=diversity_tilt_deg,
            diversity_yaw_deg=diversity_yaw_deg,
        )

        try:
            if self._params.save.format == "yaml":
                save_yaml_snapshot(
                    output_path,
                    snapshot,
                    atomic_write=self._params.save.atomic_write,
                )
            elif self._params.save.format == "json":
                self._save_json_snapshot(output_path, snapshot)
            else:
                raise MountingPipelineError("Unsupported save.format")
        except Exception:
            self._save_fail_count += 1
            return False, output_path

        self._last_save_time_ns = t_ns
        return True, output_path

    def _save_json_snapshot(
        self,
        path: str,
        snapshot: MountingSnapshotYaml,
    ) -> None:
        """Save a calibration snapshot to JSON."""
        data: dict[str, object] = snapshot_to_dict(snapshot)
        text: str = json.dumps(data, indent=2, sort_keys=True)
        self._atomic_write_text(path, text, self._params.save.atomic_write)

    def _atomic_write_text(self, path: str, text: str, atomic: bool) -> None:
        """Write text to a path, optionally atomically."""
        path_obj: Path = Path(path)
        path_obj.parent.mkdir(parents=True, exist_ok=True)
        if not atomic:
            path_obj.write_text(text, encoding="utf-8")
            return
        tmp_name: str = f".{path_obj.name}.tmp.{os.getpid()}"
        tmp_path: Path = path_obj.with_name(tmp_name)
        tmp_path.write_text(text, encoding="utf-8")
        os.replace(tmp_path, path_obj)

    def _build_snapshot_yaml(
        self,
        *,
        t_ns: int,
        diversity_tilt_deg: float | None,
        diversity_yaw_deg: float | None,
    ) -> MountingSnapshotYaml:
        """Build a MountingSnapshotYaml for persistence."""
        base_frame: str = self._params.frames.base_frame
        imu_frame: str = self._imu_frame_id or ""
        mag_frame: str = self._mag_frame_id or ""

        R_BI: MountRotationYaml = MountRotationYaml(
            quaternion_wxyz=self._state.mount.q_BI_wxyz,
            rot_cov_rad2=np.zeros((3, 3), dtype=np.float64),
        )
        R_BM: MountRotationYaml = MountRotationYaml(
            quaternion_wxyz=self._state.mount.q_BM_wxyz,
            rot_cov_rad2=np.zeros((3, 3), dtype=np.float64),
        )

        imu_yaml: ImuNuisanceYaml = ImuNuisanceYaml(
            accel_bias_mps2=self._state.imu.b_a_mps2,
            accel_A_row_major=self._state.imu.A_a.flatten(),
            accel_param_cov_row_major_12x12=np.zeros(144, dtype=np.float64),
            gyro_bias_rads=self._state.imu.b_g_rads,
            gyro_bias_cov_row_major_3x3=np.zeros(9, dtype=np.float64),
        )
        mag_yaml: MagNuisanceYaml = MagNuisanceYaml(
            offset_t=np.zeros(3, dtype=np.float64),
            offset_cov_row_major_3x3=np.zeros(9, dtype=np.float64),
            R_m_unitless2_row_major_3x3=self._state.mag.R_m_unitless2.flatten(),
            R_m0_unitless2_row_major_3x3=np.diag(
                self._params.mag.Rm_init_diag
            ).flatten(),
        )

        diversity: DiversityYaml = DiversityYaml(
            gravity_max_angle_deg=float(diversity_tilt_deg or 0.0),
            mag_proj_max_angle_deg=float(diversity_yaw_deg or 0.0),
        )

        residual_rms: float = float(self._last_update_metrics.get("rms", 0.0))
        duration_sec: float = 0.0
        if self._bootstrap_start_ns is not None:
            duration_sec = max(0.0, float(t_ns - self._bootstrap_start_ns) / 1e9)

        quality: QualityYaml = QualityYaml(
            raw_samples=self._raw_sample_count,
            steady_segments=self._segment_count,
            keyframes=len(self._keyframe_cluster.keyframes()),
            total_duration_sec=duration_sec,
            accel_residual_rms=residual_rms,
            mag_residual_rms=residual_rms,
            diversity=diversity,
        )

        flags: FlagsYaml = FlagsYaml(
            anchored=self._state.anchored,
            mag_reference_invalid=self._state.mag_reference_invalid,
            mag_disturbance_detected=False,
            mag_dir_prior_from_driver_cov=self._params.mag.use_driver_cov_as_prior,
        )
        frames: FramesYaml = FramesYaml(
            base_frame=base_frame,
            imu_frame=imu_frame,
            mag_frame=mag_frame,
        )
        return MountingSnapshotYaml(
            format_version=1,
            frames=frames,
            flags=flags,
            R_BI=R_BI,
            R_BM=R_BM,
            imu=imu_yaml,
            mag=mag_yaml,
            quality=quality,
        )

    def _build_update_report(
        self,
        *,
        did_update: bool,
        update_metrics: dict[str, Any],
        need_more_tilt: bool,
        need_more_yaw: bool,
        dropped_segments: int,
        dropped_mags: int,
    ) -> UpdateReport:
        """Build an UpdateReport from current metrics."""
        iterations: int = int(update_metrics.get("iterations", 0))
        residual_rms: float | None = None
        if "rms" in update_metrics:
            residual_rms = float(update_metrics.get("rms", 0.0))
        return UpdateReport(
            did_update=did_update,
            iterations=iterations,
            step_norm=None,
            residual_rms=residual_rms,
            need_more_tilt=need_more_tilt,
            need_more_yaw=need_more_yaw,
            dropped_segments=dropped_segments,
            dropped_mags=dropped_mags,
            message=None,
        )

    def _build_diagnostics(
        self,
        *,
        t_ns: int,
        report: UpdateReport,
        diversity_tilt_deg: float | None,
        diversity_yaw_deg: float | None,
        did_save: bool,
    ) -> Diagnostics:
        """Build a diagnostics snapshot for the current step."""
        pipeline_state: str = "bootstrapping" if self._bootstrapping else "running"
        warnings: list[str] = []
        errors: list[str] = []
        info: list[str] = []
        if did_save:
            info.append("snapshot_saved")
        if self._save_fail_count > 0:
            warnings.append("snapshot_save_failed")
        return Diagnostics(
            t_meas_ns=t_ns,
            pipeline_state=pipeline_state,
            update_report=report,
            segment_count=self._segment_count,
            keyframe_count=len(self._keyframe_cluster.keyframes()),
            diversity_tilt_deg=diversity_tilt_deg,
            diversity_yaw_deg=diversity_yaw_deg,
            warnings=tuple(warnings),
            errors=tuple(errors),
            info=tuple(info),
            dropped_packets=self._dropped_packets,
        )
