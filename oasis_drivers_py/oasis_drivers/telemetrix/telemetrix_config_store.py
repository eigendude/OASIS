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
import re
from pathlib import Path
from typing import Any
from typing import Optional


class TelemetrixConfigStore:
    """Persist Telemetrix bridge state as JSON on local disk."""

    SCHEMA_VERSION: int = 1

    def __init__(self, path: Path) -> None:
        self._path: Path = path

    @property
    def path(self) -> Path:
        """Return the JSON cache file path."""

        return self._path

    @classmethod
    def default_path(cls, com_port: str) -> Path:
        """Build a stable cache path for a Telemetrix bridge instance."""

        cache_key: str = cls._sanitize_cache_key(com_port)
        return Path.home() / ".cache" / "oasis" / f"telemetrix_{cache_key}.json"

    @classmethod
    def _sanitize_cache_key(cls, value: str) -> str:
        sanitized: str = re.sub(r"[^A-Za-z0-9._-]+", "_", value).strip("._-")
        if sanitized:
            return sanitized
        return "default"

    def load(self) -> Optional[dict[str, Any]]:
        """Load persisted cache data if it exists and is well-formed."""

        try:
            with self._path.open("r", encoding="utf-8") as cache_file:
                document: Any = json.load(cache_file)
        except FileNotFoundError:
            return None
        except (OSError, json.JSONDecodeError):
            return None

        if not isinstance(document, dict):
            return None

        return document

    def save(self, document: dict[str, object]) -> None:
        """Write cache data atomically."""

        self._path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path: Path = self._path.with_suffix(f"{self._path.suffix}.tmp")

        with temporary_path.open("w", encoding="utf-8") as cache_file:
            json.dump(document, cache_file, indent=2, sort_keys=True)
            cache_file.write("\n")

        temporary_path.replace(self._path)
