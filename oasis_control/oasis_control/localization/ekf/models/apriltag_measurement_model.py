################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
AprilTag measurement model helpers for the EKF
"""

from typing import Optional
from typing import Tuple

import numpy as np

from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import CameraInfoData


class AprilTagMeasurementModel:
    """
    Placeholder AprilTag measurement model
    """

    def __init__(self) -> None:
        self._camera_info: Optional[CameraInfoData] = None

    def update(self) -> None:
        """
        Apply an AprilTag measurement update
        """

        raise NotImplementedError("Use reprojection_residuals for AprilTag updates")

    def set_camera_info(self, camera_info: CameraInfoData) -> None:
        """
        Cache camera intrinsics for future reprojection work
        """

        self._camera_info = camera_info

    def reprojection_residuals(
        self, tag_pose_c: np.ndarray, detection: AprilTagDetection, tag_size_m: float
    ) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """
        Compute reprojection residuals and Jacobians for a detection
        """

        if self._camera_info is None:
            return None

        if len(detection.corners_px) != 8:
            return None

        tag_pose_c_array: np.ndarray = np.asarray(tag_pose_c, dtype=float).reshape(-1)
        if tag_pose_c_array.shape[0] < 6:
            return None

        z: np.ndarray = np.asarray(detection.corners_px, dtype=float)
        z_hat: Optional[np.ndarray] = self._project_tag_corners(
            tag_pose_c_array[:6], tag_size_m
        )
        if z_hat is None:
            return None

        h: Optional[np.ndarray] = self._reprojection_jacobian(
            tag_pose_c_array[:6], tag_size_m
        )
        if h is None:
            return None

        residual: np.ndarray = z - z_hat
        return residual, z_hat, h

    def pose_measurement(self, detection: AprilTagDetection) -> Optional[np.ndarray]:
        if detection.pose_world_xyz_yaw is None:
            return None
        if len(detection.pose_world_xyz_yaw) != 4:
            return None
        return np.asarray(detection.pose_world_xyz_yaw, dtype=float)

    def linearize_pose(self, state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        z_hat: np.ndarray = np.asarray(
            [state[0], state[1], state[2], state[8]], dtype=float
        )
        h: np.ndarray = np.zeros((4, 9), dtype=float)
        h[0, 0] = 1.0
        h[1, 1] = 1.0
        h[2, 2] = 1.0
        h[3, 8] = 1.0
        return z_hat, h

    def _project_tag_corners(
        self, tag_pose_c: np.ndarray, tag_size_m: float
    ) -> Optional[np.ndarray]:
        if self._camera_info is None:
            return None

        tag_corners_t: np.ndarray = self._tag_corners(tag_size_m)

        translation_c: np.ndarray = tag_pose_c[:3]
        roll: float = float(tag_pose_c[3])
        pitch: float = float(tag_pose_c[4])
        yaw: float = float(tag_pose_c[5])
        rotation_c: np.ndarray = self._rotation_matrix(roll, pitch, yaw)

        corners_c: np.ndarray = (rotation_c @ tag_corners_t.T).T + translation_c
        projected: Optional[np.ndarray] = self._project_points(
            corners_c, self._camera_info
        )
        if projected is None:
            return None

        return projected.reshape(-1)

    def _reprojection_jacobian(
        self, tag_pose_c: np.ndarray, tag_size_m: float
    ) -> Optional[np.ndarray]:
        z_hat: Optional[np.ndarray] = self._project_tag_corners(tag_pose_c, tag_size_m)
        if z_hat is None:
            return None

        jacobian: np.ndarray = np.zeros((8, 6), dtype=float)

        # Meters step used for translation numerical derivatives
        pos_eps_m: float = 1.0e-4

        # Radians step used for rotation numerical derivatives
        rot_eps_rad: float = 1.0e-4

        for index in range(6):
            delta: np.ndarray = np.zeros(6, dtype=float)
            eps: float = pos_eps_m if index < 3 else rot_eps_rad
            delta[index] = eps

            z_plus: Optional[np.ndarray] = self._project_tag_corners(
                tag_pose_c + delta, tag_size_m
            )
            z_minus: Optional[np.ndarray] = self._project_tag_corners(
                tag_pose_c - delta, tag_size_m
            )
            if z_plus is None or z_minus is None:
                return None

            jacobian[:, index] = (z_plus - z_minus) / (2.0 * eps)

        return jacobian

    def _tag_corners(self, tag_size_m: float) -> np.ndarray:
        half_size_m: float = 0.5 * tag_size_m
        return np.array(
            [
                [-half_size_m, -half_size_m, 0.0],
                [half_size_m, -half_size_m, 0.0],
                [half_size_m, half_size_m, 0.0],
                [-half_size_m, half_size_m, 0.0],
            ],
            dtype=float,
        )

    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr: float = float(np.cos(roll))
        sr: float = float(np.sin(roll))
        cp: float = float(np.cos(pitch))
        sp: float = float(np.sin(pitch))
        cy: float = float(np.cos(yaw))
        sy: float = float(np.sin(yaw))
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=float,
        )

    def _project_points(
        self, points_c: np.ndarray, camera_info: CameraInfoData
    ) -> Optional[np.ndarray]:
        if points_c.ndim != 2 or points_c.shape[1] != 3:
            return None

        k_matrix: np.ndarray = np.asarray(camera_info.k, dtype=float).reshape(3, 3)
        fx: float = float(k_matrix[0, 0])
        fy: float = float(k_matrix[1, 1])
        cx: float = float(k_matrix[0, 2])
        cy: float = float(k_matrix[1, 2])

        z_values: np.ndarray = points_c[:, 2]
        if np.any(z_values <= 0.0):
            return None

        x_norm: np.ndarray = points_c[:, 0] / z_values
        y_norm: np.ndarray = points_c[:, 1] / z_values

        distorted: Optional[Tuple[np.ndarray, np.ndarray]] = self._distort_points(
            x_norm, y_norm, camera_info
        )
        if distorted is None:
            return None

        x_dist: np.ndarray
        y_dist: np.ndarray
        x_dist, y_dist = distorted

        u_vals: np.ndarray = fx * x_dist + cx
        v_vals: np.ndarray = fy * y_dist + cy
        return np.stack((u_vals, v_vals), axis=1)

    def _distort_points(
        self, x_norm: np.ndarray, y_norm: np.ndarray, camera_info: CameraInfoData
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        model: str = camera_info.distortion_model
        coeffs: list[float] = camera_info.d
        if not model or model == "none" or not coeffs:
            return x_norm, y_norm

        if model == "plumb_bob":
            return self._distort_plumb_bob(x_norm, y_norm, coeffs)

        if model == "rational_polynomial":
            return self._distort_rational_polynomial(x_norm, y_norm, coeffs)

        return None

    def _distort_plumb_bob(
        self, x_norm: np.ndarray, y_norm: np.ndarray, coeffs: list[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        k1: float = float(coeffs[0]) if len(coeffs) > 0 else 0.0
        k2: float = float(coeffs[1]) if len(coeffs) > 1 else 0.0
        p1: float = float(coeffs[2]) if len(coeffs) > 2 else 0.0
        p2: float = float(coeffs[3]) if len(coeffs) > 3 else 0.0
        k3: float = float(coeffs[4]) if len(coeffs) > 4 else 0.0

        r2: np.ndarray = x_norm**2 + y_norm**2

        # Radial distortion scale using k1, k2, k3 coefficients
        radial: np.ndarray = 1.0 + k1 * r2 + k2 * r2**2 + k3 * r2**3

        x_tan: np.ndarray = 2.0 * p1 * x_norm * y_norm + p2 * (r2 + 2.0 * x_norm**2)
        y_tan: np.ndarray = p1 * (r2 + 2.0 * y_norm**2) + 2.0 * p2 * x_norm * y_norm
        return x_norm * radial + x_tan, y_norm * radial + y_tan

    def _distort_rational_polynomial(
        self, x_norm: np.ndarray, y_norm: np.ndarray, coeffs: list[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        k1: float = float(coeffs[0]) if len(coeffs) > 0 else 0.0
        k2: float = float(coeffs[1]) if len(coeffs) > 1 else 0.0
        p1: float = float(coeffs[2]) if len(coeffs) > 2 else 0.0
        p2: float = float(coeffs[3]) if len(coeffs) > 3 else 0.0
        k3: float = float(coeffs[4]) if len(coeffs) > 4 else 0.0
        k4: float = float(coeffs[5]) if len(coeffs) > 5 else 0.0
        k5: float = float(coeffs[6]) if len(coeffs) > 6 else 0.0
        k6: float = float(coeffs[7]) if len(coeffs) > 7 else 0.0

        r2: np.ndarray = x_norm**2 + y_norm**2
        r4: np.ndarray = r2**2
        r6: np.ndarray = r2**3

        # Rational radial distortion numerator and denominator
        radial_num: np.ndarray = 1.0 + k1 * r2 + k2 * r4 + k3 * r6
        radial_den: np.ndarray = 1.0 + k4 * r2 + k5 * r4 + k6 * r6
        radial: np.ndarray = radial_num / radial_den

        x_tan: np.ndarray = 2.0 * p1 * x_norm * y_norm + p2 * (r2 + 2.0 * x_norm**2)
        y_tan: np.ndarray = p1 * (r2 + 2.0 * y_norm**2) + 2.0 * p2 * x_norm * y_norm
        return x_norm * radial + x_tan, y_norm * radial + y_tan
