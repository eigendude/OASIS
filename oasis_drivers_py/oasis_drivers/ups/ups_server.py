################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import logging
import os
import subprocess
from typing import Optional

from oasis_msgs.msg import UPSStatus as UPSStatusMsg


class UpsServer:
    """
    Read and write to UPS (uninterruptible power supply) devices via NUT (Network
    UPS Tools)

    Dependencies:

      * nut-client (sudo apt install nut-client)

    """

    @classmethod
    def read_status(cls, logger: logging.Logger) -> Optional[UPSStatusMsg]:
        # Configure NUT
        env = os.environ.copy()
        env["NUT_QUIET_INIT_SSL"] = "true"

        try:
            result = subprocess.run(
                ["upsc", "ups"],
                capture_output=True,
                text=True,
                timeout=5.0,
                env=cls._get_nut_env(),
            )

            if result.returncode != 0:
                # Call failed, UPS is probably disconnected
                return None

            values: dict[str, str] = dict(
                line.split(": ", 1)
                for line in result.stdout.strip().split("\n")
                if ": " in line
            )

            load_total: int = int(values.get("ups.realpower.nominal", 0))

            msg = UPSStatusMsg(
                manufacturer=values.get("device.mfr", ""),
                model=values.get("device.model", ""),
                serial_number=values.get("device.serial", ""),
                input_voltage=float(values.get("input.voltage", 0.0)),
                output_voltage=float(values.get("output.voltage", 0.0)),
                battery_charge=int(values.get("battery.charge", 0)),
                battery_runtime=int(values.get("battery.runtime", 0)),
                load=float(int(values.get("ups.load", 0)) * load_total / 100),
                load_total=float(load_total),
                status=values.get("ups.status", ""),
            )
            return msg

        except Exception as e:
            logger.error(f"Failed to read UPS status: {e}")
            return None

    @classmethod
    def send_command(cls, command: str, delay: float, logger: logging.Logger) -> bool:
        # Configure NUT
        env = os.environ.copy()
        env["NUT_QUIET_INIT_SSL"] = "true"

        try:
            args: list[str] = ["upscmd", "ups", command]

            if delay > 0:
                args += [str(delay)]

            result = subprocess.run(
                args, capture_output=True, text=True, env=cls._get_nut_env()
            )

            if result.returncode != 0:
                logger.error(f"upscmd failed: {result.stderr.strip()}")
                return False

            logger.info(f"Command succeeded: {result.stdout.strip()}")
            return True

        except Exception as e:
            logger.error(f"Error sending UPS command: {e}")
            return False

    @staticmethod
    def _get_nut_env() -> dict[str, str]:
        """
        Get the environment variables for NUT.
        """
        env = os.environ.copy()
        env["NUT_QUIET_INIT_SSL"] = "true"
        return env
