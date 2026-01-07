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
IMU-driven process model stubs for the EKF
"""


class ImuProcessModel:
    """
    Placeholder process model driven by IMU data
    """

    def __init__(self) -> None:
        pass

    def predict(self, dt_s: float) -> None:
        """
        Predict the process model forward by dt seconds
        """

        # TODO: Implement continuous-time propagation and state transition
        _ = dt_s
