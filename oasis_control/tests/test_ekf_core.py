import unittest
from typing import Optional

import numpy as np

from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import EkfAprilTagDetectionUpdate
from oasis_control.localization.ekf.ekf_types import ImuSample


def _make_config(
    dt_imu_max: float = 0.05,
    apriltag_gate_d2: float = 9.49,
    accel_noise_var: float = 1.0,
    gyro_noise_var: float = 0.01,
) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=2.0,
        epsilon_wall_future=0.05,
        dt_clock_jump_max=0.5,
        dt_imu_max=dt_imu_max,
        pos_var=1.0,
        vel_var=1.0,
        ang_var=0.25,
        accel_noise_var=accel_noise_var,
        gyro_noise_var=gyro_noise_var,
        gravity_mps2=9.80665,
        max_dt_sec=0.05,
        checkpoint_interval_sec=0.5,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
        apriltag_gate_d2=apriltag_gate_d2,
        tag_size_m=0.162,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=10.0,
        tag_landmark_prior_sigma_rot_rad=3.141592653589793,
    )


class EkfCoreTests(unittest.TestCase):
    def test_find_checkpoint_includes_equality(self) -> None:
        config: EkfConfig = _make_config()
        core: EkfCore = EkfCore(config)
        core._ensure_initialized(0.0)
        core._save_checkpoint(1.0)

        checkpoint: Optional[object] = core._find_checkpoint(1.0)

        self.assertIsNotNone(checkpoint)
        self.assertEqual(1.0, checkpoint.t_meas)

    def test_propagate_skips_large_imu_gap(self) -> None:
        config: EkfConfig = _make_config(dt_imu_max=0.1)
        core: EkfCore = EkfCore(config)
        core._ensure_initialized(0.0)

        imu_sample: ImuSample = ImuSample(
            frame_id="imu",
            angular_velocity_rps=[0.0, 0.0, 0.0],
            linear_acceleration_mps2=[1.0, 0.0, 0.0],
            angular_velocity_cov=[0.0] * 9,
            linear_acceleration_cov=[0.0] * 9,
        )
        core._last_imu = imu_sample
        core._last_imu_time = 0.0
        core._t_frontier = 0.0

        x_before: np.ndarray = core.state()
        p_before: np.ndarray = core.covariance()
        core._propagate_if_needed(1.0)
        x_after: np.ndarray = core.state()
        p_after: np.ndarray = core.covariance()

        self.assertTrue(np.allclose(x_before, x_after))
        self.assertTrue(np.allclose(p_before, p_after))
        self.assertEqual(1.0, core.frontier_time())

    def test_process_noise_scaling(self) -> None:
        config: EkfConfig = _make_config(accel_noise_var=2.0, gyro_noise_var=3.0)
        core: EkfCore = EkfCore(config)
        q: np.ndarray = core._process_noise(0.5)

        pos_expected: float = 2.0 * (0.5**3)
        vel_expected: float = 2.0 * 0.5
        ang_expected: float = 3.0 * 0.5

        for index in range(3):
            self.assertAlmostEqual(pos_expected, float(q[index, index]))
            self.assertAlmostEqual(vel_expected, float(q[index + 3, index + 3]))
            self.assertAlmostEqual(ang_expected, float(q[index + 6, index + 6]))

    def test_apriltag_mahalanobis_gate_rejects(self) -> None:
        config: EkfConfig = _make_config(apriltag_gate_d2=1.0)
        core: EkfCore = EkfCore(config)
        core._ensure_initialized(0.0)

        detection: AprilTagDetection = AprilTagDetection(
            family="tag36h11",
            tag_id=5,
            det_index_in_msg=0,
            corners_px=[0.0] * 8,
            pose_world_xyz_yaw=[10.0, 0.0, 0.0, 0.0],
            decision_margin=1.0,
            homography=[0.0] * 9,
        )

        x_before: np.ndarray = core.state()
        p_before: np.ndarray = core.covariance()
        update: EkfAprilTagDetectionUpdate = core._update_with_apriltag_detection(
            detection, "camera", 0.0
        )
        x_after: np.ndarray = core.state()
        p_after: np.ndarray = core.covariance()

        self.assertFalse(update.update.accepted)
        self.assertEqual("mahalanobis_gate", update.update.reject_reason)
        self.assertEqual(1.0, update.update.gate_d2_threshold)
        self.assertGreater(update.update.maha_d2, 1.0)
        self.assertTrue(np.allclose(x_before, x_after))
        self.assertTrue(np.allclose(p_before, p_after))


if __name__ == "__main__":
    unittest.main()
