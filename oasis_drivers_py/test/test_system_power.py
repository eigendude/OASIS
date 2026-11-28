################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import math
from pathlib import Path
from unittest.mock import Mock

import pytest

from oasis_drivers.system.system_power import SystemPowerReader


def domain(root: Path, dirname: str, name: str) -> Path:
    zone: Path = root / dirname
    zone.mkdir()
    (zone / "name").write_text(name + "\n")
    (zone / "max_energy_range_uj").write_text("100000000")
    energy: Path = zone / "energy_uj"
    energy.write_text("80000000")
    return energy


@pytest.fixture
def clock() -> Mock:
    return Mock(return_value=100.0)


def test_discovers_psys_and_caches_range(tmp_path: Path, clock: Mock) -> None:
    domain(tmp_path, "intel-rapl:0", "package-0")
    domain(tmp_path, "intel-rapl:0:1", "core")
    (tmp_path / "intel-rapl:1").mkdir()
    energy: Path = domain(tmp_path, "intel-rapl:7", "psys")
    reader: SystemPowerReader = SystemPowerReader(tmp_path, clock)
    assert reader.read() is None

    # A steady-state sample needs only the psys energy attribute
    (energy.parent / "name").unlink()
    (energy.parent / "max_energy_range_uj").unlink()
    energy.write_text("87500000")
    clock.return_value += 0.5
    assert reader.read() == 15.0
    clock.return_value += 0.25
    assert reader.read() == 0.0


def test_rollover(tmp_path: Path, clock: Mock) -> None:
    energy: Path = domain(tmp_path, "intel-rapl:3", "psys")
    reader: SystemPowerReader = SystemPowerReader(tmp_path, clock)
    assert reader.read() is None
    energy.write_text("2500000")
    clock.return_value += 0.75
    assert reader.read() == 30.0


def test_missing_psys_reprobes_after_backoff(tmp_path: Path, clock: Mock) -> None:
    domain(tmp_path, "intel-rapl:1", "package-0")
    domain(tmp_path, "other:0", "psys")
    reader: SystemPowerReader = SystemPowerReader(tmp_path, clock)
    assert reader.read() is None
    clock.return_value += 0.5
    assert reader.read() is None

    domain(tmp_path, "intel-rapl-mmio:9", "psys")
    clock.return_value += 0.5
    assert reader.read() is None
    clock.return_value += 60.0
    assert reader.read() is None
    clock.return_value += 0.5
    assert reader.read() == 0.0


@pytest.mark.parametrize("bad_range", [b"", b"0", b"-1", b"bad"])
def test_invalid_range(tmp_path: Path, clock: Mock, bad_range: bytes) -> None:
    energy: Path = domain(tmp_path, "intel-rapl:2", "psys")
    maximum: Path = energy.parent / "max_energy_range_uj"
    maximum.write_bytes(bad_range)
    reader: SystemPowerReader = SystemPowerReader(tmp_path, clock)
    assert reader.read() is None
    clock.return_value += 1.0
    assert reader.read() is None

    maximum.write_bytes(b"100000000")
    clock.return_value += 60.0
    assert reader.read() is None
    clock.return_value += 0.5
    assert reader.read() == 0.0


@pytest.mark.parametrize("bad_energy", [b"-1", b"100000001", b"", b"1_0", b"\xff"])
def test_invalid_energy_resets_baseline(
    tmp_path: Path, clock: Mock, bad_energy: bytes
) -> None:
    energy: Path = domain(tmp_path, "intel-rapl:2", "psys")
    reader: SystemPowerReader = SystemPowerReader(tmp_path, clock)
    assert reader.read() is None
    energy.write_bytes(bad_energy)
    clock.return_value += 0.5
    assert reader.read() is None

    energy.write_text("1000000")
    clock.return_value += 0.5
    assert reader.read() is None
    energy.write_text("3000000")
    clock.return_value += 0.5
    assert reader.read() == 4.0


@pytest.mark.parametrize("bad_time", [100.0, 99.0, math.nan, math.inf, -math.inf])
def test_invalid_time_resets_baseline(
    tmp_path: Path, clock: Mock, bad_time: float
) -> None:
    domain(tmp_path, "intel-rapl:2", "psys")
    reader: SystemPowerReader = SystemPowerReader(tmp_path, clock)
    assert reader.read() is None
    clock.return_value = bad_time
    assert reader.read() is None
    clock.return_value = 101.0
    assert reader.read() is None
    clock.return_value += 0.5
    assert reader.read() == 0.0


@pytest.mark.parametrize("failure", [PermissionError(), OSError("read failed")])
def test_read_error_resets_baseline(
    tmp_path: Path, clock: Mock, monkeypatch: pytest.MonkeyPatch, failure: OSError
) -> None:
    energy: Path = domain(tmp_path, "intel-rapl:2", "psys")
    reader: SystemPowerReader = SystemPowerReader(tmp_path, clock)
    assert reader.read() is None
    clock.return_value += 0.5
    with monkeypatch.context() as patch:
        patch.setattr(Path, "open", Mock(side_effect=failure))
        assert reader.read() is None

    energy.write_text("0")
    clock.return_value += 60.0
    assert reader.read() is None
    energy.write_text("10000000")
    clock.return_value += 0.5
    assert reader.read() == 20.0


def test_domain_reappears_at_new_index(tmp_path: Path, clock: Mock) -> None:
    energy: Path = domain(tmp_path, "intel-rapl:2", "psys")
    reader: SystemPowerReader = SystemPowerReader(tmp_path, clock)
    assert reader.read() is None
    energy.parent.rename(tmp_path / "removed")
    clock.return_value += 0.5
    assert reader.read() is None
    domain(tmp_path, "intel-rapl:8", "psys")
    clock.return_value += 60.0
    assert reader.read() is None
    clock.return_value += 0.5
    assert reader.read() == 0.0


def test_unreadable_psys_does_not_hide_another_interface(
    tmp_path: Path, clock: Mock
) -> None:
    domain(tmp_path, "intel-rapl-mmio:0", "psys").unlink()
    energy: Path = domain(tmp_path, "intel-rapl:2", "psys")
    reader: SystemPowerReader = SystemPowerReader(tmp_path, clock)
    assert reader.read() is None
    energy.write_text("85000000")
    clock.return_value += 0.5
    assert reader.read() == 10.0


@pytest.mark.parametrize("new_name", ["psys", "package-0"])
def test_reused_zone_index(tmp_path: Path, clock: Mock, new_name: str) -> None:
    energy: Path = domain(tmp_path, "intel-rapl:2", "psys")
    reader: SystemPowerReader = SystemPowerReader(tmp_path, clock)
    assert reader.read() is None
    energy.parent.rename(tmp_path / "removed")
    replacement: Path = domain(tmp_path, "intel-rapl:2", new_name)
    replacement.write_text("0")
    clock.return_value += 0.5
    assert reader.read() is None
    replacement.write_text("1000000")
    clock.return_value += 0.5
    if new_name == "psys":
        assert reader.read() == 2.0
    else:
        assert reader.read() is None
