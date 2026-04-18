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
# Compact AHRS runtime status for diagnostics and bring-up
#

# AHRS inputs are valid enough for the current lightweight runtime
uint8 STATUS_OK = 0

# No acceptable IMU sample has been observed yet
uint8 STATUS_WAITING_FOR_IMU = 1

# No acceptable gravity sample has been observed yet
uint8 STATUS_WAITING_FOR_GRAVITY = 2

# IMU samples are arriving with an unexpected frame_id
uint8 STATUS_BAD_IMU_FRAME = 3

# Gravity samples are arriving with an unexpected frame_id
uint8 STATUS_BAD_GRAVITY_FRAME = 4

# Fixed mounting transform is not available in the current skeleton
uint8 STATUS_MOUNTING_UNAVAILABLE = 5

# Status timestamp and primary AHRS output frame
#
# header.frame_id is expected to be the configured world frame
#
std_msgs/Header header

# Current AHRS runtime status code
uint8 status

# Number of IMU packets accepted by the current frame policy
#
# Units: packets
#
uint32 accepted_imu_count

# Number of gravity packets accepted by the current frame policy
#
# Units: packets
#
uint32 accepted_gravity_count

# Number of IMU packets rejected by the current frame policy
#
# Units: packets
#
uint32 rejected_imu_count

# Number of gravity packets rejected by the current frame policy
#
# Units: packets
#
uint32 rejected_gravity_count

# Number of stale IMU packets dropped by timestamp policy
#
# Units: packets
#
uint32 dropped_stale_imu_count

# Number of stale gravity packets dropped by timestamp policy
#
# Units: packets
#
uint32 dropped_stale_gravity_count

# Number of gravity consistency checks rejected by runtime policy
#
# Units: packets
#
uint32 gravity_rejection_count

# Gravity residual summary for the current AHRS state
#
# Units: implementation-defined
#
# NaN when no residual has been computed yet
#
float64 gravity_residual_norm

# Gravity gating summary for the current AHRS state
#
# Units: implementation-defined
#
# NaN when no gating distance has been computed yet
#
float64 gravity_mahalanobis_distance

# True once at least one gravity packet has been accepted
bool has_gravity

# True once a fixed mounting transform is available to the node
bool has_mounting

# True when the latest gravity consistency check passed the AHRS gate
bool gravity_gated_in

# True when the latest gravity consistency check was rejected by policy
bool gravity_rejected

# Number of TF lookup failures while sourcing the fixed mounting transform
#
# Units: packets
#
uint32 transform_lookup_failure_count

# Number of invalid fixed mounting transforms returned by TF
#
# Units: packets
#
uint32 invalid_mounting_transform_count

# Short text from the latest mounting lookup or transform validity problem
string last_mounting_lookup_error

# Short machine-facing reason from the latest gravity consistency rejection
string last_gravity_rejection_reason

# Short implementation-facing status text for logs and UIs
string status_text
