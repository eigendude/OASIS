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

import json
import sys
import unittest
from pathlib import Path
from typing import Dict
from typing import List


ROOT: Path = Path(__file__).resolve().parents[4]
PACKAGE_ROOT: Path = ROOT / "oasis_control"
PACKAGE_SRC: Path = PACKAGE_ROOT / "oasis_control"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
if "oasis_control" in sys.modules:
    module_path_raw = getattr(sys.modules["oasis_control"], "__path__", None)
    module_paths: List[str] = []
    if module_path_raw is not None:
        module_paths = list(module_path_raw)
        if str(PACKAGE_SRC) not in module_paths:
            module_paths.append(str(PACKAGE_SRC))
            sys.modules["oasis_control"].__path__ = module_paths

from oasis_control.localization.ahrs.config.ahrs_params import AhrsParams


class TestAhrsParams(unittest.TestCase):
    """Tests for the AhrsParams container."""

    def _spd(self, value: float) -> List[List[float]]:
        return [
            [value, 0.0, 0.0],
            [0.0, value, 0.0],
            [0.0, 0.0, value],
        ]

    def test_defaults_validate(self) -> None:
        """Defaults produce a valid configuration."""
        params: AhrsParams = AhrsParams.defaults()
        params.validate()

    def test_from_dict_rejects_unknown_key(self) -> None:
        """from_dict rejects unknown parameters deterministically."""
        params: Dict[str, object] = {
            "alpha": 0.5,
            "beta": 0.1,
        }
        with self.assertRaises(ValueError) as context:
            AhrsParams.from_dict(params)
        self.assertEqual(str(context.exception), "unknown parameter: beta")

    def test_validate_rejects_negative_sigma(self) -> None:
        """Validate rejects negative noise intensities."""
        params: AhrsParams = AhrsParams.defaults()
        params = AhrsParams(
            **{
                **params.as_dict(),
                "sigma_w_v": -0.1,
            }
        )
        with self.assertRaises(ValueError) as context:
            params.validate()
        self.assertEqual(str(context.exception), "sigma_w_v must be >= 0")

    def test_validate_rejects_bad_rmax_diag(self) -> None:
        """Validate rejects R_max with diagonal smaller than R_min."""
        params: AhrsParams = AhrsParams.defaults()
        params = AhrsParams(
            **{
                **params.as_dict(),
                "R_min": self._spd(2.0e-9),
                "R_max": self._spd(1.0e-9),
            }
        )
        with self.assertRaises(ValueError) as context:
            params.validate()
        self.assertEqual(
            str(context.exception),
            "R_max diagonal must be >= R_min diagonal",
        )

    def test_as_dict_json_serializable(self) -> None:
        """as_dict returns JSON-serializable data."""
        params: AhrsParams = AhrsParams.defaults()
        payload: Dict[str, object] = params.as_dict()
        json.dumps(payload)


if __name__ == "__main__":
    unittest.main()
