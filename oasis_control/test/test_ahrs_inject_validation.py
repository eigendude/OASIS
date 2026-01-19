################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

from dataclasses import dataclass

import pytest

from oasis_control.localization.ahrs.ahrs_inject import inject_error_state
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_state import default_nominal_state


@dataclass(frozen=True)
class _LayoutStub:
    dim: int
    _sl_p: slice
    _sl_v: slice
    _sl_theta: slice
    _sl_omega: slice
    _sl_bg: slice
    _sl_ba: slice
    _sl_aa: slice
    _sl_xi_bi: slice
    _sl_xi_bm: slice
    _sl_g: slice
    _sl_m: slice

    def sl_p(self) -> slice:
        return self._sl_p

    def sl_v(self) -> slice:
        return self._sl_v

    def sl_theta(self) -> slice:
        return self._sl_theta

    def sl_omega(self) -> slice:
        return self._sl_omega

    def sl_bg(self) -> slice:
        return self._sl_bg

    def sl_ba(self) -> slice:
        return self._sl_ba

    def sl_Aa(self) -> slice:
        return self._sl_aa

    def sl_xi_bi(self) -> slice:
        return self._sl_xi_bi

    def sl_xi_bm(self) -> slice:
        return self._sl_xi_bm

    def sl_g(self) -> slice:
        return self._sl_g

    def sl_m(self) -> slice:
        return self._sl_m


def _build_layout_stub(aa_len: int, xi_bi_len: int) -> _LayoutStub:
    start: int = 0
    sl_p: slice = slice(start, start + 3)
    start += 3
    sl_v: slice = slice(start, start + 3)
    start += 3
    sl_theta: slice = slice(start, start + 3)
    start += 3
    sl_omega: slice = slice(start, start + 3)
    start += 3
    sl_bg: slice = slice(start, start + 3)
    start += 3
    sl_ba: slice = slice(start, start + 3)
    start += 3
    sl_aa: slice = slice(start, start + aa_len)
    start += aa_len
    sl_xi_bi: slice = slice(start, start + xi_bi_len)
    start += xi_bi_len
    sl_xi_bm: slice = slice(start, start + 6)
    start += 6
    sl_g: slice = slice(start, start + 3)
    start += 3
    sl_m: slice = slice(start, start + 3)
    start += 3

    return _LayoutStub(
        dim=start,
        _sl_p=sl_p,
        _sl_v=sl_v,
        _sl_theta=sl_theta,
        _sl_omega=sl_omega,
        _sl_bg=sl_bg,
        _sl_ba=sl_ba,
        _sl_aa=sl_aa,
        _sl_xi_bi=sl_xi_bi,
        _sl_xi_bm=sl_xi_bm,
        _sl_g=sl_g,
        _sl_m=sl_m,
    )


def _default_state() -> AhrsNominalState:
    return default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )


def test_inject_error_state_rejects_bad_aa_length() -> None:
    layout: _LayoutStub = _build_layout_stub(aa_len=8, xi_bi_len=6)
    state: AhrsNominalState = _default_state()
    delta_x: list[float] = [0.0] * layout.dim

    with pytest.raises(ValueError, match="delta_aa must have length 9"):
        inject_error_state(layout, state, delta_x)


def test_inject_error_state_rejects_bad_xi_length() -> None:
    layout: _LayoutStub = _build_layout_stub(aa_len=9, xi_bi_len=5)
    state: AhrsNominalState = _default_state()
    delta_x: list[float] = [0.0] * layout.dim

    with pytest.raises(ValueError, match="delta_xi_bi must have length 6"):
        inject_error_state(layout, state, delta_x)
