################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Telemetry about disk partitions
#

# Device name (e.g. /dev/sda1 from devfs)
string device_name

# Filesystem type
string filesystem

# Mount point of the partition
string mount_point

# Options used to mount the partition
string mount_options

# Total patition size, in bytes
uint64 disk_total

# Used space of the partition, in bytes
uint64 disk_used

# Free space of the partition, in bytes
uint64 disk_free

# Percent utilization of the partition (used / total)
float32 disk_percent
