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
# Status for the boot-time IMU mounting calibration solve
#

# Calibration has not started
uint8 STATUS_IDLE = 0

# Collecting samples in the boot window
uint8 STATUS_COLLECTING = 1

# Solving the fixed mounting transform from accepted samples
uint8 STATUS_SOLVING = 2

# Calibration completed successfully
uint8 STATUS_SUCCEEDED = 3

# Calibration completed without a usable result
uint8 STATUS_FAILED = 4

# Measurement time and the frame being calibrated
std_msgs/Header header

# Current boot calibration phase
uint8 status

# Configured boot calibration window duration
#
# Units: s
#
float64 boot_window_duration_sec

# Number of samples accepted into the solve window
#
# Units: samples
#
uint32 accepted_sample_count

# Number of samples rejected during boot calibration
#
# Units: samples
#
uint32 rejected_sample_count

# True when the current window satisfies the stationary / low-rate policy
bool stationary_accepted

# Gravity-direction consistency metric over the accepted window
#
# Units: implementation-defined
#
float64 gravity_consistency_metric

# Fused-attitude consistency metric over the accepted window
#
# Units: implementation-defined
#
float64 fused_attitude_consistency_metric

# True when a valid fixed transform has been solved
bool success

# Short implementation-facing status text for logs and UIs
string status_text
