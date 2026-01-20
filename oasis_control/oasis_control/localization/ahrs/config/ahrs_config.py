################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""High-level configuration container for the AHRS core.

Responsibility:
    Provide a cohesive configuration object that aggregates AhrsParams and
    exposes settings to the filter, models, and timing components.

Inputs/outputs:
    - Inputs: AhrsParams or configuration dictionaries.
    - Outputs: validated, immutable configuration used by the core.

Dependencies:
    - Uses AhrsParams for schema and validation.

Determinism:
    Configuration is immutable once constructed for deterministic behavior.
"""


class AhrsConfig:
    """AHRS configuration container and validation rules.

    Purpose:
        Aggregate all AHRS parameters into a single object passed throughout
        the core, ensuring validation and immutability.

    Public API (to be implemented):
        - from_params(params)
        - validate()
        - as_dict()

    Data contract:
        - Contains all fields defined in AhrsParams.
        - May precompute derived values (e.g., noise variances).

    Frames and units:
        - Frame identifiers are stored as strings.
        - Numeric fields use Units-defined scales.

    Determinism and edge cases:
        - Construction must not read ROS params or clocks.
        - Invalid parameters should raise validation errors.

    Equations:
        - No equations; configuration only.

    Numerical stability notes:
        - Ensure derived covariances are SPD.

    Suggested unit tests:
        - as_dict round-trip preserves values.
        - validate rejects missing required fields.
    """

    pass
