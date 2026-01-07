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
Helpers for building EKF update reports
"""

from __future__ import annotations

from std_msgs.msg import Header

from oasis_control.localization.ekf.ekf_types import EkfAprilTagDetectionUpdate
from oasis_control.localization.ekf.ekf_types import EkfAprilTagUpdateData
from oasis_control.localization.ekf.ekf_types import EkfMatrix
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_msgs.msg import EkfAprilTagDetectionReport
from oasis_msgs.msg import EkfAprilTagUpdateReport
from oasis_msgs.msg import EkfUpdateReport
from oasis_msgs.msg import Matrix


class UpdateReporter:
    """
    Build update report messages from EKF payloads
    """

    def __init__(self) -> None:
        pass

    def build_update_report(
        self, update: EkfUpdateData, update_seq: int, update_index_in_stamp: int
    ) -> EkfUpdateReport:
        report: EkfUpdateReport = EkfUpdateReport()
        report.header = self._build_header(update.t_meas, update.frame_id)
        report.sensor = update.sensor
        report.frame_id = update.frame_id
        report.update_seq = update_seq
        report.update_index_in_stamp = update_index_in_stamp
        report.accepted = update.accepted
        report.reject_reason = update.reject_reason
        report.z_dim = update.z_dim
        report.z = list(update.z)
        report.z_hat = list(update.z_hat)
        report.nu = list(update.nu)
        report.r = self._matrix_msg(update.r)
        report.s_hat = self._matrix_msg(update.s_hat)
        report.s = self._matrix_msg(update.s)
        report.maha_d2 = update.maha_d2
        report.gate_d2_threshold = update.gate_d2_threshold
        report.reproj_rms_px = update.reproj_rms_px
        return report

    def build_apriltag_report(
        self, update: EkfAprilTagUpdateData, update_seq: int
    ) -> EkfAprilTagUpdateReport:
        report: EkfAprilTagUpdateReport = EkfAprilTagUpdateReport()
        report.header = self._build_header(update.t_meas, update.frame_id)
        report.update_seq = update_seq
        report.num_detections_in_msg = len(update.detections)

        sorted_detections: list[EkfAprilTagDetectionUpdate] = sorted(
            update.detections, key=lambda det: (det.family, det.tag_id)
        )

        accepted_count: int = 0
        report.detections = []
        for order_index, detection in enumerate(sorted_detections):
            if detection.update.accepted:
                accepted_count += 1

            detection_report: EkfAprilTagDetectionReport = EkfAprilTagDetectionReport()
            detection_report.family = detection.family
            detection_report.id = detection.tag_id
            detection_report.det_index_in_msg = detection.det_index_in_msg
            detection_report.order_index = order_index
            detection_report.report = self.build_update_report(
                detection.update, update_seq, order_index
            )
            report.detections.append(detection_report)

        report.num_detections_accepted = accepted_count
        return report

    def _build_header(self, timestamp: float, frame_id: str) -> Header:
        header: Header = Header()
        header.stamp = EkfTime.from_seconds(timestamp)
        header.frame_id = frame_id
        return header

    def _matrix_msg(self, matrix: EkfMatrix) -> Matrix:
        matrix_msg: Matrix = Matrix()
        matrix_msg.rows = matrix.rows
        matrix_msg.cols = matrix.cols
        matrix_msg.data = list(matrix.data)
        return matrix_msg
