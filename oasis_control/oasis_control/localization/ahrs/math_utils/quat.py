################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS quat definitions."""


class Quaternion:
    """
    Quaternion math utilities for world-to-body rotations.

    The AHRS uses quaternions that rotate vectors from frame {W} to {B}:

        v_B = R(q_WB) * v_W

    Composition follows Hamilton product with the convention:

        q_AB = q_CB ⊗ q_AC
        R(q_AB) = R(q_CB) * R(q_AC)

    The continuous-time attitude kinematics use the angular-rate matrix:

        Ω(ω) = [ 0   -ωx  -ωy  -ωz
                 ωx   0    ωz  -ωy
                 ωy  -ωz   0    ωx
                 ωz   ωy  -ωx   0 ]

        q̇ = 0.5 * Ω(ω) * q
    """

    pass
