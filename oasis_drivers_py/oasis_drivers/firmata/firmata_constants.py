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

    # Extended command set using sysex (0-127/0x00-0x7F)
    MEMORY_CONFIG = 0x64  # Enable/disable reporting of memory statistics
    MEMORY_DATA = 0x65  # Receive a memory statistics report
