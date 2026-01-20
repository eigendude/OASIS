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
from typing import List
from typing import cast


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
        """from_params converts t_buffer_sec to nanoseconds."""
        params: AhrsParams = AhrsParams.defaults()
        config: AhrsConfig = AhrsConfig.from_params(params)
        expected_ns: int = int(params.t_buffer_sec * 1_000_000_000 + 0.5)
        self.assertEqual(config.t_buffer_ns, expected_ns)

    def test_from_params_mapping(self) -> None:
        """from_params accepts a mapping input."""
        config: AhrsConfig = AhrsConfig.from_params({"t_buffer_sec": 3.0})
        self.assertEqual(config.params.t_buffer_sec, 3.0)
        self.assertEqual(config.t_buffer_ns, 3_000_000_000)

    def test_as_dict_round_trip(self) -> None:
        """as_dict emits JSON-serializable output."""
        config: AhrsConfig = AhrsConfig.from_params(AhrsParams.defaults())
        payload: dict[str, object] = config.as_dict()
        json.dumps(payload)
        self.assertEqual(payload["t_buffer_ns"], config.t_buffer_ns)
        params_payload: dict[str, object] = cast(dict[str, object], payload["params"])
        self.assertEqual(params_payload["t_buffer_sec"], config.params.t_buffer_sec)


if __name__ == "__main__":
    unittest.main()
