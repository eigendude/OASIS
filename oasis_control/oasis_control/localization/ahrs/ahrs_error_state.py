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
Explicit error-state layout for AHRS covariance bookkeeping

The error-state stores small deviations from the nominal state so the filter
can track uncertainty with linearized dynamics. The covariance matrix uses
this stacked error vector, so a deterministic layout and stable naming are
critical for mapping blocks and publishing diagnostics.

The extrinsic pose blocks use an SE(3) tangent representation. Each block is
stacked as (delta rho, delta theta), where delta rho is the small translation
error and delta theta is the small-angle rotation error.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ErrorBlock:
    """
    Descriptor for a contiguous block in the stacked error-state vector

    Fields:
        name: Human-readable name for the block
        dim: Dimension of the block in the error vector
        start: Starting index of the block in the stacked vector
    """

    name: str
    dim: int
    start: int

    @property
    def sl(self) -> slice:
        """
        Slice for this block inside the stacked error-state vector
        """

        return slice(self.start, self.start + self.dim)


class AhrsErrorStateLayout:
    """
    Deterministic error-state layout for the AHRS covariance matrix
    """

    def __init__(self) -> None:
        order: list[str] = []
        blocks: dict[str, ErrorBlock] = {}
        start: int = 0

        for key, name, dim in (
            ("p", "p_wb", 3),
            ("v", "v_wb", 3),
            ("theta", "theta_wb", 3),
            ("omega", "omega_wb", 3),
            ("bg", "b_g", 3),
            ("ba", "b_a", 3),
            ("Aa", "A_a", 9),
            ("xi_bi", "xi_bi", 6),
            ("xi_bm", "xi_bm", 6),
            ("g", "g_w", 3),
            ("m", "m_w", 3),
        ):
            blocks[key] = ErrorBlock(name=name, dim=dim, start=start)
            order.append(key)
            start += dim

        self.blocks: dict[str, ErrorBlock] = blocks
        self.order: list[str] = order
        self.dim: int = start

    def sl_p(self) -> slice:
        """
        Slice for position error delta p
        """

        return self.blocks["p"].sl

    def sl_v(self) -> slice:
        """
        Slice for velocity error delta v
        """

        return self.blocks["v"].sl

    def sl_theta(self) -> slice:
        """
        Slice for attitude small-angle error delta theta
        """

        return self.blocks["theta"].sl

    def sl_omega(self) -> slice:
        """
        Slice for angular-rate error delta omega
        """

        return self.blocks["omega"].sl

    def sl_bg(self) -> slice:
        """
        Slice for gyro bias error delta b_g
        """

        return self.blocks["bg"].sl

    def sl_ba(self) -> slice:
        """
        Slice for accel bias error delta b_a
        """

        return self.blocks["ba"].sl

    def sl_Aa(self) -> slice:
        """
        Slice for accel matrix error delta A_a
        """

        return self.blocks["Aa"].sl

    def sl_xi_bi(self) -> slice:
        """
        Slice for IMU extrinsic error delta xi_bi
        """

        return self.blocks["xi_bi"].sl

    def sl_xi_bm(self) -> slice:
        """
        Slice for magnetometer extrinsic error delta xi_bm
        """

        return self.blocks["xi_bm"].sl

    def sl_g(self) -> slice:
        """
        Slice for gravity vector error delta g_w
        """

        return self.blocks["g"].sl

    def sl_m(self) -> slice:
        """
        Slice for magnetic field error delta m_w
        """

        return self.blocks["m"].sl


def error_state_names(layout: AhrsErrorStateLayout) -> list[str]:
    """
    Build per-element error-state names in the stacked layout order
    """

    names: list[str] = []
    axes: tuple[str, str, str] = ("x", "y", "z")

    for key in layout.order:
        if key == "p":
            names.extend([f"δp_wb_{axis}" for axis in axes])
        elif key == "v":
            names.extend([f"δv_wb_{axis}" for axis in axes])
        elif key == "theta":
            names.extend([f"δtheta_wb_{axis}" for axis in axes])
        elif key == "omega":
            names.extend([f"δomega_wb_{axis}" for axis in axes])
        elif key == "bg":
            names.extend([f"δb_g_{axis}" for axis in axes])
        elif key == "ba":
            names.extend([f"δb_a_{axis}" for axis in axes])
        elif key == "Aa":
            for row in range(3):
                for col in range(3):
                    names.append(f"δA_a_r{row}c{col}")
        elif key == "xi_bi":
            names.extend([f"δxi_bi_rho_{axis}" for axis in axes])
            names.extend([f"δxi_bi_theta_{axis}" for axis in axes])
        elif key == "xi_bm":
            names.extend([f"δxi_bm_rho_{axis}" for axis in axes])
            names.extend([f"δxi_bm_theta_{axis}" for axis in axes])
        elif key == "g":
            names.extend([f"δg_w_{axis}" for axis in axes])
        elif key == "m":
            names.extend([f"δm_w_{axis}" for axis in axes])
        else:
            raise ValueError(f"Unknown error-state block key: {key}")

    if len(names) != layout.dim:
        raise ValueError(
            f"Error-state names length {len(names)} does not match "
            f"layout dim {layout.dim}"
        )

    return names
