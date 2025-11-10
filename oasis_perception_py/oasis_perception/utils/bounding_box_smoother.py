################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import math
from typing import Optional


################################################################################
# Bounding box class
################################################################################


class BoundingBoxSmoother:
    """
    Smooth bounding boxes and apply normalized padding.

    All inputs and outputs are in normalized [0..1] coordinates.
    """

    def __init__(
        self,
        *,
        base_alpha: float = 0.3,
        padding: float = 0.02,
        velocity_threshold: float = 0.05,
    ) -> None:
        """
        Initialize the smoother.

        :param base_alpha: Base smoothing factor ∈ [0..1]. Lower = smoother.
        :param padding: Normalized padding to apply on each side of the box.
        :param velocity_threshold: Normalized center‐to‐center movement that
                                   triggers α→1.0 (no lag).
        """
        self.base_alpha: float = base_alpha
        self.padding: float = padding
        self.velocity_threshold: float = velocity_threshold
        self._prev_box: Optional[tuple[float, float, float, float]] = None

    def update(
        self,
        raw_box: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        """
        Smooth the raw box and return a padded, smoothed box, all in [0..1].

        :param raw_box: (x_min, y_min, x_max, y_max) in normalized coords.
        :return: (x_min, y_min, x_max, y_max) smoothed & padded, clamped to [0..1].
        """
        # Unpack
        x0, y0, x1, y1 = raw_box
        # Compute center + half‐widths
        cx = (x0 + x1) * 0.5
        cy = (y0 + y1) * 0.5
        hw = (x1 - x0) * 0.5
        hh = (y1 - y0) * 0.5

        # First frame: no smoothing
        if self._prev_box is None:
            scx, scy, shw, shh = cx, cy, hw, hh
        else:
            # Unpack previous
            px0, py0, px1, py1 = self._prev_box
            pcx = (px0 + px1) * 0.5
            pcy = (py0 + py1) * 0.5
            phw = (px1 - px0) * 0.5
            phh = (py1 - py0) * 0.5

            # Measure normalized velocity
            disp = math.hypot(cx - pcx, cy - pcy)

            # Adapt alpha: no lag if disp ≥ threshold
            if disp >= self.velocity_threshold:
                alpha = 1.0
            else:
                alpha = self.base_alpha + (1 - self.base_alpha) * (
                    disp / self.velocity_threshold
                )

            # Smooth each component
            scx = alpha * cx + (1 - alpha) * pcx
            scy = alpha * cy + (1 - alpha) * pcy
            shw = alpha * hw + (1 - alpha) * phw
            shh = alpha * hh + (1 - alpha) * phh

        # Store for next call
        nx0 = scx - shw
        ny0 = scy - shh
        nx1 = scx + shw
        ny1 = scy + shh
        self._prev_box = (nx0, ny0, nx1, ny1)

        # Apply normalized padding
        px0 = nx0 - self.padding
        py0 = ny0 - self.padding
        px1 = nx1 + self.padding
        py1 = ny1 + self.padding

        # Clamp to [0..1]
        x_min = max(0.0, px0)
        y_min = max(0.0, py0)
        x_max = min(1.0, px1)
        y_max = min(1.0, py1)

        return (x_min, y_min, x_max, y_max)

    def get_smoothed_box(self) -> tuple[float, float, float, float]:
        """
        Return the last smoothed & padded box (normalized), without re-smoothing.
        """
        if self._prev_box is None:
            raise RuntimeError("No previous box: call update() first.")
        raw_box = self._prev_box
        # Reapply padding & clamp
        px0 = raw_box[0] - self.padding
        py0 = raw_box[1] - self.padding
        px1 = raw_box[2] + self.padding
        py1 = raw_box[3] + self.padding
        return (
            max(0.0, px0),
            max(0.0, py0),
            min(1.0, px1),
            min(1.0, py1),
        )
