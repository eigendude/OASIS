################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import os
import re
import shutil
import stat
import subprocess
from typing import Optional


class DisplayServer:
    """
    Server to control displays. Uses the following technologies:

      * DPMS (Display Power Management Signaling)
      * DDC/CI (Display Data Channel Command Interface)
      * CEC (Consumer Electronics Control)

    Optional dependencies:

      * cec-client
      * ddcutil
      * vbetool - Needs setuid, run "sudo chmod u+s /usr/sbin/vbetool"

    For vbetool, starting with Ubuntu 20.04, you need to run this command
    after startup:

      sudo mount -o remount,exec /dev

    """

    # Class variable to hold all detected CEC adapters
    _cec_adapters: list[int] = []

    @staticmethod
    def ensure_dpms() -> None:
        """
        Check whether DPMS control via vbetool is available, and raise an
        exception if not.

        Verifies that the 'vbetool' binary is installed and has its setuid bit
        set.

        :return: True if vbetool is present (and warns if not setuid), False if
                 missing
        """
        # 1) Locate the binary
        vbetool_path: Optional[str] = shutil.which("vbetool")
        if vbetool_path is None:
            raise Exception('Missing "vbetool". Install with: sudo apt install vbetool')

        # 2) Check for the setuid bit
        try:
            st_mode: int = os.stat(vbetool_path).st_mode
        except OSError as err:
            raise Exception(f'Cannot stat "{vbetool_path}": {err}')

        if not (st_mode & stat.S_ISUID):
            raise Exception(
                f'"{vbetool_path}" exists but is not setuid. '
                f"Grant permissions with: sudo chmod u+s {vbetool_path}"
            )

    @staticmethod
    def set_dpms(power_mode: bool) -> None:
        """
        Turn DPMS on or off via vbetool.

        :param power_mode: True to power ON, False to power OFF.

        :raises Exception: if vbetool fails or is not installed.
        """
        power_mode_string: str = "on" if power_mode else "off"

        try:
            vbetool_proc: subprocess.CompletedProcess[bytes] = subprocess.run(
                ["vbetool", "dpms", power_mode_string],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            retcode: int = vbetool_proc.returncode
            if retcode != 0:
                # Likely missing setuid
                raise Exception(
                    f'vbetool returned {retcode}, try running "sudo chmod u+s /usr/sbin/vbetool". '
                    # Could also be an LRMI permissions error, see:
                    #   https://bugs.launchpad.net/ubuntu/+source/vbetool/+bug/1875240
                    'If that doesn\'t work, try "sudo mount -o remount,exec /dev" on every boot'
                )
        except FileNotFoundError:
            raise Exception('Missing vbetool, run "sudo apt install vbetool"')

    @staticmethod
    def detect_displays() -> str:
        """
        Detect attached displays via DDC/CI and return a report.

        Runs `ddcutil detect`, then summarizes each display's I²C bus,
        DRM connector name, and model.

        :returns: Multi-line string report

        :raises Exception: if ddcutil is missing or `ddcutil detect` fails
        """
        # 1) Run `ddcutil detect`
        try:
            proc: subprocess.CompletedProcess[str] = subprocess.run(
                ["ddcutil", "detect"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError:
            raise Exception("Missing ddcutil; install with: sudo apt install ddcutil")

        # 2) Handle non-zero exit
        if proc.returncode != 0:
            err = proc.stderr.strip() or proc.stdout.strip()
            raise Exception(f"ddcutil detect failed (code {proc.returncode}):\n{err}")

        out: str = proc.stdout

        # 3) Build report lines
        report_lines: list[str] = ["ddcutil detect output:"]
        report_lines += [f"  {line}" for line in out.splitlines()]

        # 4) Parse key fields
        buses: list[str] = re.findall(r"I2C bus:\s*(/dev/i2c-\d+)", out)
        connectors: list[str] = re.findall(r"DRM connector:\s*(\S+)", out)
        models: list[str] = re.findall(r"Model:\s*(.+)", out)

        # 5) Summarize each display
        count: int = max(len(buses), len(connectors), len(models))
        for i in range(count):
            bus = buses[i] if i < len(buses) else "Unknown"
            connector = connectors[i] if i < len(connectors) else "Unknown"
            model = models[i] if i < len(models) else "Unknown"
            report_lines.append(
                f"Display {i + 1}: bus={bus}, connector={connector}, model={model}"
            )

        return "\n".join(report_lines)

    @staticmethod
    def set_brightness(brightness: int) -> None:
        """
        Set the panel brightness via DDC/CI using ddcutil.

        :param brightness: Desired brightness percentage (0–100)

        :raises Exception: if ddcutil is missing or ddcutil fails
        """
        # 1) Clamp input
        if brightness < 0:
            brightness = 0
        elif brightness > 100:
            brightness = 100

        # 2) Detect I²C bus
        try:
            detect: subprocess.CompletedProcess[str] = subprocess.run(
                ["ddcutil", "detect"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                text=True,
            )
        except FileNotFoundError:
            raise Exception('Missing ddcutil, install with "sudo apt install ddcutil"')

        out: str = detect.stdout + detect.stderr
        match: Optional[re.Match[str]] = re.search(r"I2C bus:\s*/dev/i2c-(\d+)", out)
        if not match:
            raise Exception(
                "Unable to determine I2C bus from ddcutil detect output:\n"
                + out.splitlines()[0]
            )

        bus: str = match.group(1)

        # 3) Set brightness
        cmd: list[str] = ["ddcutil", "--bus", bus, "setvcp", "0x10", str(brightness)]
        proc: subprocess.CompletedProcess[str] = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )

        if proc.returncode != 0:
            raise Exception(
                f"ddcutil failed (code {proc.returncode})\n"
                f"STDOUT: {proc.stdout.strip()}\n"
                f"STDERR: {proc.stderr.strip()}"
            )

    @staticmethod
    def ensure_cec() -> None:
        """
        Check whether CEC control via cec-client is available and a CEC device
        is present, and raise an exception if not.

        :raises Exception: if cec-client is missing or no CEC device is found
        """
        # 1) Ensure cec-client binary exists
        if shutil.which("cec-client") is None:
            raise Exception(
                'Missing "cec-client". Install with: sudo apt install cec-utils'
            )

        # 2) Run `cec-client -l` to list adapters
        try:
            proc: subprocess.CompletedProcess[str] = subprocess.run(
                ["cec-client", "-l"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
        except FileNotFoundError:
            raise Exception(
                "cec-client not found; install with: sudo apt install cec-utils"
            )

        output: str = proc.stdout + proc.stderr

        # 3) Parse adapter indices (lines like 'device:              1')
        adapters = [int(idx) for idx in re.findall(r"device:\s+(\d+)", output)]
        if not adapters:
            raise Exception(f"No CEC adapters found:\n{output.strip()}")

        DisplayServer._cec_adapters = adapters

    @staticmethod
    def set_cec_power(power_mode: bool) -> None:
        """
        Send CEC power command on every adapter detected on initialization.

        :param power_mode: True to power ON, False to power OFF
        """
        if not DisplayServer._cec_adapters:
            return

        # Determine CEC command
        cec_cmd: str
        if power_mode:
            # TVs often ignore “on”, so use “activate-source”
            cec_cmd = "as"
        else:
            cec_cmd = "standby 0"

        # Send to all adapters
        for adapter in DisplayServer._cec_adapters:
            try:
                subprocess.run(
                    ["cec-client", "-s", "-d", str(adapter)],
                    input=(cec_cmd + "\n").encode(),
                    check=True,
                )
            except subprocess.CalledProcessError as err:
                # Log and continue on failure for other adapters
                raise Exception(f"CEC adapter {adapter} failed: {err}")
