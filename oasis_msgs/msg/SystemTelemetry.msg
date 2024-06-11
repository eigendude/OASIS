################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Telemetry about running processes and system utilization
#

# Timestamp and frame_id
std_msgs/Header header

# The time when the system was booted (Unix timestamp)
float64 bootstrap_time

# CPU utilization, as a percent
float32 cpu_utilization

# CPU temperature (C)
float64 cpu_temperature

# CPU frequency (GHz)
float64 cpu_frequency_ghz

# CPU core count (physical cores)
uint32 cpu_physical_core_count

# CPU core count (logical cores)
uint32 cpu_logical_core_count

# RAM utilization, as a percent
float32 memory_utilization

# Disk partitions
DiskPartition[] disk_partitions

# Network interfaces
NetworkInterface[] network_interfaces

# True if a battery is present, false otherwise
bool has_battery

# Battery percent
float32 battery_percent

# Time remaining on the battery (minutes)
float64 battery_remaining_mins

# True if the power is plugged, false if operating on battery power
bool power_plugged
