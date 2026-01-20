################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS extrinsics model definitions."""


class ExtrinsicsModel:
    """
    Extrinsic calibration model for IMU and magnetometer frames.

    Extrinsics are stored as SE(3) transforms:

        T_BI: IMU → body
        T_BM: mag → body

    Perturbations use a 6D tangent vector δξ = [δρ; δθ], and covariance blocks are
    updated using SE(3) adjoint mappings.
    """

    pass
