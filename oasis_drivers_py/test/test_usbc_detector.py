################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from pathlib import Path

from oasis_drivers.system.usbc import usbc_max_current_ma


def test_maximum_tracks_connected_usb_c_devices(tmp_path: Path) -> None:
    devices: Path = tmp_path / "bus/usb/devices"
    for name, power in (("2-1", "500mA"), ("2-3", "896mA"), ("3-5", "900mA")):
        device: Path = devices / name
        (device / "port").mkdir(parents=True)
        (device / "bMaxPower").write_text(power)
    (devices / "2-1/port/connector").mkdir()
    (devices / "2-3/port/connector").mkdir()
    (devices / "3-5/port/location").write_text("0x80000005")

    assert usbc_max_current_ma(tmp_path) == 896
    (devices / "2-3/bMaxPower").unlink()
    assert usbc_max_current_ma(tmp_path) == 500
    (devices / "2-1/bMaxPower").unlink()
    assert usbc_max_current_ma(tmp_path) is None


def test_nuc_usb_c_socket_without_connector_link(tmp_path: Path) -> None:
    device: Path = tmp_path / "bus/usb/devices/2-2"
    (device / "port").mkdir(parents=True)
    (device / "port/location").write_text("0x80000002\n")
    (device / "bMaxPower").write_text("896mA\n")
    board: Path = tmp_path / "class/dmi/id/board_name"
    board.parent.mkdir(parents=True)
    board.write_text("NUC14RVS\n")
    assert usbc_max_current_ma(tmp_path) == 896
    board.write_text("another board\n")
    assert usbc_max_current_ma(tmp_path) is None
