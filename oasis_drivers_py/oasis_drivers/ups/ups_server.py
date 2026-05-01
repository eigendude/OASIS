################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import math
import os
import subprocess
from typing import Optional

import rclpy.logging

from oasis_msgs.msg import UPSStatus as UPSStatusMsg


class UpsServer:
    """
    Read and write to UPS (uninterruptible power supply) devices via NUT (Network
    UPS Tools)

    Dependencies:

      * nut-client (sudo apt install nut-client)

    """

    @classmethod
    def read_status(cls, logger: rclpy.logging.RcutilsLogger) -> Optional[UPSStatusMsg]:
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

            load_total: int = cls._int_value(values, "ups.realpower.nominal")
            ups_load_percent: int = cls._int_value(values, "ups.load")

            # Convert NUT load percentage to watts using nominal real power
            load_watts: float = float(ups_load_percent * load_total / 100.0)

            msg: UPSStatusMsg = UPSStatusMsg(
                manufacturer=values.get("device.mfr", ""),
                model=values.get("device.model", ""),
                serial_number=values.get("device.serial", ""),
                input_voltage=cls._float_value(values, "input.voltage", 0.0),
                output_voltage=cls._float_value(values, "output.voltage", 0.0),
                battery_voltage=cls._float_value(values, "battery.voltage", math.nan),
                battery_current=cls._float_value(values, "battery.current", math.nan),
                battery_temperature=cls._float_value(
                    values,
                    "battery.temperature",
                    math.nan,
                ),
                battery_charge=cls._int_value(values, "battery.charge"),
                battery_runtime=cls._int_value(values, "battery.runtime"),
                load=load_watts,
                load_total=float(load_total),
                status=values.get("ups.status", ""),
                battery_type=values.get("battery.type", ""),
            )
            return msg

        except Exception as e:
            logger.error(f"Failed to read UPS status: {e}")
            return None

    @classmethod
    def send_command(
        cls, command: str, delay: float, logger: rclpy.logging.RcutilsLogger
    ) -> bool:
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

    @staticmethod
    def _float_value(
        values: dict[str, str],
        key: str,
        default: float,
    ) -> float:
        try:
            value: float = float(values[key])
        except (KeyError, TypeError, ValueError):
            return default

        if not math.isfinite(value):
            return default

        return value

    @staticmethod
    def _int_value(
        values: dict[str, str],
        key: str,
        default: int = 0,
    ) -> int:
        try:
            return int(float(values[key]))
        except (KeyError, TypeError, ValueError):
            return default
