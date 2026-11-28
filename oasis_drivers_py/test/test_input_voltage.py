################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import subprocess
import sys
import time
from unittest.mock import Mock

import pytest

from oasis_drivers.system.input_voltage import InputVoltageReader


COMMAND: list[str] = ["vcgencmd", "pmic_read_adc", "EXT5V_V"]


@pytest.fixture
def run(monkeypatch: pytest.MonkeyPatch) -> Mock:
    runner: Mock = Mock()
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(subprocess, "run", runner)
    return runner


def reading(output: bytes) -> subprocess.CompletedProcess[bytes]:
    return subprocess.CompletedProcess(COMMAND, 0, output)


def test_samples_fresh_channel_voltage(run: Mock) -> None:
    run.side_effect = [
        reading(b"     EXT5V_V volt(24)=4.71546000V\n"),
        reading(b"HDMI_V volt(22)=4.9V\n     EXT5V_V volt(24)=4.50000000V\n"),
    ]
    reader: InputVoltageReader = InputVoltageReader()

    assert reader.read() == pytest.approx(4.71546)
    assert reader.read() == 4.5
    assert run.call_args.args == (COMMAND,)
    assert not run.call_args.kwargs.get("shell", False)
    assert 0 < run.call_args.kwargs["timeout"] < 0.5


def test_macos_skips_command(run: Mock, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys, "platform", "darwin")
    reader: InputVoltageReader = InputVoltageReader()

    assert reader.read() is None
    run.assert_not_called()


@pytest.mark.parametrize(
    "failure",
    [
        FileNotFoundError(),
        PermissionError(),
        reading(b'error=1 error_msg="Command not registered"'),
        reading(b"HDMI_V volt(22)=4.7V"),
    ],
)
def test_unsupported_reader_only_probes_once(
    run: Mock, failure: Exception | subprocess.CompletedProcess[bytes]
) -> None:
    run.side_effect = [failure]
    reader: InputVoltageReader = InputVoltageReader()

    assert reader.read() is None
    assert reader.read() is None
    run.assert_called_once()


@pytest.mark.parametrize(
    "failure",
    [
        subprocess.TimeoutExpired(COMMAND, 0.2),
        subprocess.CalledProcessError(1, COMMAND),
        OSError("Resource temporarily unavailable"),
        reading(b"EXT5V_V volt(24)=brokenV"),
        reading(b"\xff"),
    ],
)
def test_inconclusive_probe_backs_off_then_recovers(
    run: Mock,
    monkeypatch: pytest.MonkeyPatch,
    failure: Exception | subprocess.CompletedProcess[bytes],
) -> None:
    clock: Mock = Mock(return_value=100.0)
    monkeypatch.setattr(time, "monotonic", clock)
    run.side_effect = [failure, reading(b"EXT5V_V volt(24)=4.7V")]
    reader: InputVoltageReader = InputVoltageReader()

    assert reader.read() is None
    clock.return_value += 0.5
    assert reader.read() is None
    run.assert_called_once()
    clock.return_value += 60.0
    assert reader.read() == 4.7


@pytest.mark.parametrize(
    "failure",
    [
        subprocess.TimeoutExpired(COMMAND, 0.2),
        subprocess.CalledProcessError(1, COMMAND),
        reading(b"EXT5V_V volt(24)=brokenV"),
        reading(b"EXT5V_V volt(24)=4.7A"),
        reading(b"EXT5V_V volt(24)=4.7V trailing garbage"),
        reading(b"\xff"),
    ],
)
def test_failed_sample_on_supported_hardware_recovers(
    run: Mock, failure: Exception | subprocess.CompletedProcess[bytes]
) -> None:
    run.side_effect = [
        reading(b"EXT5V_V volt(24)=4.7V"),
        failure,
        reading(b"EXT5V_V volt(24)=4.6V"),
    ]
    reader: InputVoltageReader = InputVoltageReader()

    assert reader.read() == 4.7
    assert reader.read() is None
    assert reader.read() == 4.6
