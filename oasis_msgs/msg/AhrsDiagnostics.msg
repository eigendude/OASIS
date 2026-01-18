################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Health / replay / drop diagnostics for the AHRS node.
#
# Conventions
# - header.stamp = t_filter (frontier)
# - header.frame_id = world frame_id
#

# Frontier timestamp and world frame_id
std_msgs/Header header

# Monotonic diagnostic sequence index
uint64 diag_seq

# The filter's current frontier timestamp in seconds (for convenience)
float64 t_filter_sec

#
# Buffer / replay stats
#

# Number of time nodes currently retained
uint32 buffer_node_count

# Approximate time span covered by the buffer (s)
float32 buffer_span_sec

# True if an out-of-order message triggered replay on the most recent callback
bool replay_happened

#
# Drop counters (monotonic since node start)
#

uint64 dropped_missing_stamp
uint64 dropped_future_stamp
uint64 dropped_too_old
uint64 dropped_nan_cov
uint64 dropped_imu_coverage_gap
uint64 dropped_clock_jump_reset

#
# Most recent reset reason (empty if no reset yet)
#
string last_reset_reason
