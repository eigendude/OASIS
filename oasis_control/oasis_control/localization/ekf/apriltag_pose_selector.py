################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Helpers for selecting AprilTag pose detections deterministically
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class AprilTagPoseDetectionCandidate:
    """
    Candidate AprilTag pose detection for selection

    Fields:
        family: AprilTag family name such as tag36h11
        tag_id: AprilTag identifier within the family
        det_index_in_msg: Index in the incoming detection array
        frame_id: Frame identifier for the pose measurement
        source_topic: ROS topic name that provided the detection
        p_world_base_m: Position of base in world frame in meters, XYZ order
        q_world_base_xyzw: Quaternion of base in world frame, XYZW order
        covariance: 6x6 pose covariance in world frame, row-major
        decision_margin: Optional detector decision margin for gating
        pose_error: Optional detector pose error for tie-breaking
    """

    family: str
    tag_id: int
    det_index_in_msg: int
    frame_id: str
    source_topic: str
    p_world_base_m: list[float]
    q_world_base_xyzw: list[float]
    covariance: list[float]
    decision_margin: Optional[float]
    pose_error: Optional[float]


def select_best_apriltag_candidate(
    candidates: list[AprilTagPoseDetectionCandidate],
) -> Optional[AprilTagPoseDetectionCandidate]:
    """
    Select the best detection candidate deterministically
    """

    if not candidates:
        return None

    def _sort_key(
        candidate: AprilTagPoseDetectionCandidate,
    ) -> tuple[int, float, float, int]:
        decision_margin: float = (
            float(candidate.decision_margin)
            if candidate.decision_margin is not None
            else float("-inf")
        )
        pose_error: float = (
            float(candidate.pose_error)
            if candidate.pose_error is not None
            else float("inf")
        )
        return (
            candidate.tag_id,
            -decision_margin,
            pose_error,
            candidate.det_index_in_msg,
        )

    return min(candidates, key=_sort_key)
