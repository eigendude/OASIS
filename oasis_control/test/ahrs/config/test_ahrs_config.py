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

from oasis_control.localization.ahrs.config.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.config.ahrs_params import AhrsParams


class TestAhrsConfig(unittest.TestCase):
    """Tests for the AhrsConfig container."""

    def test_from_params_converts_buffer(self) -> None:
        """from_params converts buffer seconds into nanoseconds once."""
        params: AhrsParams = AhrsParams.defaults()
        params = AhrsParams(**{**params.as_dict(), "t_buffer_sec": 1.5})
        config: AhrsConfig = AhrsConfig.from_params(params)
        self.assertEqual(config.t_buffer_ns, 1_500_000_000)
        config.validate()

    def test_as_dict_json_serializable(self) -> None:
        """as_dict returns JSON-serializable data."""
        config: AhrsConfig = AhrsConfig.from_params(AhrsParams.defaults())
        payload: Dict[str, object] = config.as_dict()
        json.dumps(payload)

    def test_validate_rejects_mismatched_buffer(self) -> None:
        """Validate rejects a mismatched buffer conversion."""
        params: AhrsParams = AhrsParams.defaults()
        config: AhrsConfig = AhrsConfig.from_params(params)
        config = AhrsConfig(**{**config.as_dict(), "t_buffer_ns": 1})
        with self.assertRaises(ValueError) as context:
            config.validate()
        self.assertEqual(
            str(context.exception),
            "t_buffer_ns must match t_buffer_sec",
        )


if __name__ == "__main__":
    unittest.main()
