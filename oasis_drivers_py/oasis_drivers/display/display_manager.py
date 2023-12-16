#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import subprocess


class DisplayManager:
    """
    Control power to displays. Currently only works for laptops.

    Dependencies:

      * vbetool - Needs setuid, run "sudo chmod u+s /usr/sbin/vbetool"

    Starting with Ubuntu 20.04, you need to run this command after startup:

      sudo mount -o remount,exec /dev

    """

    @staticmethod
    def call_vbetool(power_mode: bool, logger) -> None:  # TODO: logger type
        power_mode_string: str = "on" if power_mode else "off"

        try:
            vbetool_proc: subprocess.CompletedProcess = subprocess.run(
                ["vbetool", "dpms", power_mode_string],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            retcode: int = vbetool_proc.returncode
            if retcode != 0:
                # Likely missing setuid
                logger.error(
                    f'vbetool returned {retcode}, try running "sudo chmod u+s /usr/sbin/vbetool"'
                )
                # Could also be an LRMI permissions error, see:
                #   https://bugs.launchpad.net/ubuntu/+source/vbetool/+bug/1875240
                logger.error(
                    'If that doesn\'t work, try "sudo mount -o remount,exec /dev" on every boot',
                )
            else:
                logger.info(f"Power {power_mode_string} success")
        except FileNotFoundError:
            logger.error('Missing vbetool, run "sudo apt install vbetool"')
