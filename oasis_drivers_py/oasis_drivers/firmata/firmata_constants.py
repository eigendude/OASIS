################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class FirmataConstants:
    """
    This class contains a set of constants for Firmata use.
    """

    # Pin modes
    CPU_FAN_PWM: int = 0x11  # Pin is connected to the PWM wire of a CPU fan
    CPU_FAN_TACH: int = 0x12  # Pin is connected to the tachometer wire of a CPU fan

    # Extended command set using sysex (0-127/0x00-0x7F)
    CPU_FAN_RPM: int = 0x53  # Sysex message reporting CPU fan speed in RPM
    MEMORY_CONFIG: int = 0x66  # Enable/disable reporting of memory statistics
    MEMORY_DATA: int = 0x67  # Receive a memory statistics report
