#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import collections


################################################################################
# System types
################################################################################

Battery = collections.namedtuple(
    "Battery",
    [
        "battery_percent",
        "battery_remaining_time",
        "power_plugged",
    ],
)

DiskPartition = collections.namedtuple(
    "DiskPartition",
    [
        "device_name",
        "filesystem",
        "mount_point",
        "mount_options",
        "disk_total",
        "disk_used",
        "disk_free",
        "disk_percent",
    ],
)

NetworkAddress = collections.namedtuple(
    "NetworkAddress",
    [
        "family",
        "address",
        "netmask",
    ],
)

NetworkInterface = collections.namedtuple(
    "NetworkInterface",
    [
        "name",
        "provider",
        "bytes_sent",
        "bytes_received",
        "addresses",
    ],
)
