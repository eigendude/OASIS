################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Tests for the TF publication policy engine."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import NDArray

from oasis_control.localization.mounting.tf.tf_publisher import PublishedTransform
from oasis_control.localization.mounting.tf.tf_publisher import TfPublisher
from oasis_control.localization.mounting.tf.tf_publisher import TfPublisherError


def _identity_rotation() -> NDArray[np.float64]:
    """Return an identity rotation matrix."""
    return np.eye(3, dtype=np.float64)


def _yaw_rotation(angle_rad: float) -> NDArray[np.float64]:
    """Return a rotation matrix about +Z by the given angle."""
    cos_angle: float = float(np.cos(angle_rad))
    sin_angle: float = float(np.sin(angle_rad))
    return np.array(
        [
            [cos_angle, -sin_angle, 0.0],
            [sin_angle, cos_angle, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def test_tf_publisher_dynamic_publishing() -> None:
    """Emit dynamic transforms on each update when enabled."""
    publisher: TfPublisher = TfPublisher(
        base_frame="base_link",
        imu_frame="imu_link",
        mag_frame="mag_link",
        publish_dynamic=True,
        publish_static_when_stable=False,
    )
    transforms: list[PublishedTransform] = publisher.update(
        t_ns=0,
        R_BI=_yaw_rotation(0.3),
        R_BM=_yaw_rotation(-0.2),
        is_stable=False,
        saved=False,
    )
    assert len(transforms) == 2
    for transform in transforms:
        assert not transform.is_static
        assert transform.parent_frame == "base_link"
        assert transform.child_frame in {"imu_link", "mag_link"}
        np.testing.assert_allclose(
            transform.translation_m,
            np.zeros(3, dtype=np.float64),
        )


def test_tf_publisher_static_on_transition() -> None:
    """Emit static transforms when stability is reached."""
    publisher: TfPublisher = TfPublisher(
        base_frame="base_link",
        imu_frame="imu_link",
        mag_frame="mag_link",
        publish_dynamic=True,
        publish_static_when_stable=True,
    )
    publisher.update(
        t_ns=0,
        R_BI=_identity_rotation(),
        R_BM=_identity_rotation(),
        is_stable=False,
        saved=False,
    )
    transforms: list[PublishedTransform] = publisher.update(
        t_ns=1,
        R_BI=_identity_rotation(),
        R_BM=_identity_rotation(),
        is_stable=True,
        saved=False,
    )
    static_transforms: list[PublishedTransform] = [
        transform for transform in transforms if transform.is_static
    ]
    assert len(static_transforms) == 2


def test_tf_publisher_republish_on_save() -> None:
    """Republish static transforms when the calibration file is saved."""
    publisher: TfPublisher = TfPublisher(
        base_frame="base_link",
        imu_frame="imu_link",
        mag_frame="mag_link",
        publish_dynamic=False,
        publish_static_when_stable=True,
        republish_static_on_save=True,
    )
    publisher.update(
        t_ns=0,
        R_BI=_identity_rotation(),
        R_BM=_identity_rotation(),
        is_stable=True,
        saved=False,
    )
    transforms: list[PublishedTransform] = publisher.update(
        t_ns=1,
        R_BI=_identity_rotation(),
        R_BM=_identity_rotation(),
        is_stable=True,
        saved=True,
    )
    assert len(transforms) == 2
    assert all(transform.is_static for transform in transforms)


def test_tf_publisher_save_requires_stability() -> None:
    """Avoid publishing static transforms before stability even if saved."""
    publisher: TfPublisher = TfPublisher(
        base_frame="base_link",
        imu_frame="imu_link",
        mag_frame="mag_link",
        publish_dynamic=False,
        publish_static_when_stable=True,
        republish_static_on_save=True,
    )
    transforms_unstable: list[PublishedTransform] = publisher.update(
        t_ns=0,
        R_BI=_identity_rotation(),
        R_BM=_identity_rotation(),
        is_stable=False,
        saved=True,
    )
    assert not transforms_unstable

    transforms_stable: list[PublishedTransform] = publisher.update(
        t_ns=1,
        R_BI=_identity_rotation(),
        R_BM=_identity_rotation(),
        is_stable=True,
        saved=False,
    )
    assert len(transforms_stable) == 2
    assert all(transform.is_static for transform in transforms_stable)

    transforms_republish: list[PublishedTransform] = publisher.update(
        t_ns=2,
        R_BI=_identity_rotation(),
        R_BM=_identity_rotation(),
        is_stable=True,
        saved=True,
    )
    assert len(transforms_republish) == 2
    assert all(transform.is_static for transform in transforms_republish)


def test_tf_publisher_frame_validation() -> None:
    """Reject empty frame names."""
    with pytest.raises(TfPublisherError):
        TfPublisher(
            base_frame="",
            imu_frame="imu",
            mag_frame="mag",
        )


def test_tf_publisher_quaternion_normalization() -> None:
    """Ensure published quaternions are unit normalized."""
    publisher: TfPublisher = TfPublisher(
        base_frame="base_link",
        imu_frame="imu_link",
        mag_frame="mag_link",
        publish_dynamic=True,
        publish_static_when_stable=False,
    )
    transforms: list[PublishedTransform] = publisher.update(
        t_ns=0,
        R_BI=_identity_rotation(),
        R_BM=_identity_rotation(),
        is_stable=False,
        saved=False,
    )
    for transform in transforms:
        norm: float = float(np.linalg.norm(transform.quaternion_wxyz))
        assert np.isclose(norm, 1.0)
        np.testing.assert_allclose(
            transform.translation_m,
            np.zeros(3, dtype=np.float64),
        )
