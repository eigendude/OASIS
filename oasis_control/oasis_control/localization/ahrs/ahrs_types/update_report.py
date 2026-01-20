################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS update report definitions."""


class UpdateReport:
    """
    Per-measurement update report for innovation statistics.

    Stores measurement z, prediction z_hat, innovation ν, noise covariance R,
    innovation covariance S, and gating decisions. The report is published for
    both accepted and rejected updates.
    """

    pass
