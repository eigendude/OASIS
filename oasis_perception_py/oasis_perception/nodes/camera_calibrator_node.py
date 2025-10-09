################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import functools
import socket
from typing import Any
from typing import Type

import cv2
import cv_bridge
import message_filters
import numpy as np
import rclpy.publisher
from camera_calibration.calibrator import ChessboardInfo
from camera_calibration.calibrator import ImageDrawable
from camera_calibration.calibrator import Patterns
from camera_calibration.camera_calibrator import CalibrationNode
from image_transport_py import ImageTransport
from message_filters import ApproximateTimeSynchronizer
from sensor_msgs.msg import Image as ImageMsg


################################################################################
# ROS parameters
################################################################################


# Default node name
NODE_NAME: str = "camera_calibrator"

# Topic names
CALIBRATION_TOPIC: str = "calibration"


################################################################################
# Calibration parameters
################################################################################


DEFAULT_CAMERA_NAME: str = socket.gethostname().replace("-", "_")
DEFAULT_PATTERN: str = "chessboard"
DEFAULT_SIZE: list[str] = ["8x6"]
DEFAULT_SQUARE: list[float] = [0.025]
DEFAULT_CHARUCO_MARKER_SIZE: list[float] = []
DEFAULT_ARUCO_DICT: list[str] = []
DEFAULT_APPROXIMATE: float = 0.0
DEFAULT_SERVICE_CHECK: bool = False
DEFAULT_QUEUE_SIZE: int = 1
DEFAULT_FIX_PRINCIPAL_POINT: bool = False
DEFAULT_FIX_ASPECT_RATIO: bool = False
DEFAULT_ZERO_TANGENT_DIST: bool = False
DEFAULT_K_COEFFICIENTS: int = 2
DEFAULT_FISHEYE_RECOMPUTE_EXTRINSICSTS: bool = False
DEFAULT_FISHEYE_FIX_SKEW: bool = False
DEFAULT_FISHEYE_FIX_PRINCIPAL_POINT: bool = False
DEFAULT_FISHEYE_K_COEFFICIENTS: int = 4
DEFAULT_FISHEYE_CHECK_CONDITIONS: bool = False
DEFAULT_DISABLE_CALIB_CB_FAST_CHECK: bool = False
DEFAULT_MAX_CHESSBOARD_SPEED: int = -1


################################################################################
# Checkerboard parameters
################################################################################


#
# The checkerboard pattern in this package's calibration/ folder was generated
# using https://calib.io/pages/camera-calibration-pattern-generator with the
# following parameters:
#
#   - Target Type: Chessboard
#   - Board Width [mm]: 297
#   - Board Height [mm]: 210
#   - Rows: 7
#   - Columns: 9
#   - Checker Width [mm]: 25
#   - Find Maker: No
#   - Radon Checkers: No
#


################################################################################
# Helper functions
################################################################################


def options_valid_charuco(
    pattern: str,
    size: list[str],
    square: list[float],
    charuco_sizes: list[float],
    aruco_dicts: list[str],
    parser: Any,
) -> bool:
    """
    Validate the options for charuco pattern.

    :param pattern: The pattern type
    :param size: The size of the chessboard
    :param square: The size of the squares
    :param charuco_sizes: The size of the charuco markers
    :param aruco_dicts: The aruco dictionary names
    :param parser: The argument parser

    :return: True if the options are valid, False otherwise

    :raises ValueError: If the options are not valid
    """
    if pattern != "charuco":
        return False

    n: int = len(size)
    if not (n == len(square) == len(charuco_sizes) == len(aruco_dicts)):
        raise ValueError(
            "For charuco, --size, --square, --charuco_marker_size and --aruco_dict must have same length"
        )

    return True


################################################################################
# ROS node
################################################################################


class CameraCalibratorNode(CalibrationNode):
    def __init__(self) -> None:
        # Build the list of ChessboardInfo up front
        boards: list[ChessboardInfo]
        if DEFAULT_PATTERN == "charuco":
            options_valid_charuco(
                DEFAULT_PATTERN,
                DEFAULT_SIZE,
                DEFAULT_SQUARE,
                DEFAULT_CHARUCO_MARKER_SIZE,
                DEFAULT_ARUCO_DICT,
                None,
            )
            boards = [
                ChessboardInfo("charuco", *map(int, sz.split("x")), sq, ms, ad)
                for sz, sq, ms, ad in zip(
                    DEFAULT_SIZE,
                    DEFAULT_SQUARE,
                    DEFAULT_CHARUCO_MARKER_SIZE,
                    DEFAULT_ARUCO_DICT,
                )
            ]
        else:
            boards = [
                ChessboardInfo(DEFAULT_PATTERN, *map(int, sz.split("x")), sq)
                for sz, sq in zip(DEFAULT_SIZE, DEFAULT_SQUARE)
            ]

        # Pick synchronizer class
        sync_cls: Type[Any] = (
            message_filters.TimeSynchronizer
            if DEFAULT_APPROXIMATE <= 0.0
            else functools.partial(
                ApproximateTimeSynchronizer, slop=DEFAULT_APPROXIMATE
            )
        )

        # Build pinhole flags
        calib_flags: int = 0
        if DEFAULT_FIX_PRINCIPAL_POINT:
            calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        if DEFAULT_FIX_ASPECT_RATIO:
            calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
        if DEFAULT_ZERO_TANGENT_DIST:
            calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
        if DEFAULT_K_COEFFICIENTS > 3:
            calib_flags |= cv2.CALIB_RATIONAL_MODEL
        # Fix‑unused K flags...
        for k_level in range(DEFAULT_K_COEFFICIENTS):
            calib_flags |= getattr(cv2, f"CALIB_FIX_K{k_level + 1}")

        # Build fisheye flag
        fisheye_flags: int = 0
        if DEFAULT_FISHEYE_FIX_PRINCIPAL_POINT:
            fisheye_flags |= cv2.fisheye.CALIB_FIX_PRINCIPAL_POINT
        if DEFAULT_FISHEYE_FIX_SKEW:
            fisheye_flags |= cv2.fisheye.CALIB_FIX_SKEW
        if DEFAULT_FISHEYE_RECOMPUTE_EXTRINSICSTS:
            fisheye_flags |= cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        if DEFAULT_FISHEYE_CHECK_CONDITIONS:
            fisheye_flags |= cv2.fisheye.CALIB_CHECK_COND
        for k_level in range(DEFAULT_FISHEYE_K_COEFFICIENTS):
            fisheye_flags |= getattr(cv2.fisheye, f"CALIB_FIX_K{k_level + 1}")

        # Checkerboard fast‑check
        cb_flags: int = (
            0 if DEFAULT_DISABLE_CALIB_CB_FAST_CHECK else cv2.CALIB_CB_FAST_CHECK
        )

        # Pattern enum
        pat: int = {
            "chessboard": Patterns.Chessboard,
            "circles": Patterns.Circles,
            "acircles": Patterns.ACircles,
            "charuco": Patterns.ChArUco,
        }[DEFAULT_PATTERN]

        # Call the upstream constructor
        super().__init__(
            NODE_NAME,
            boards,
            DEFAULT_SERVICE_CHECK,
            sync_cls,
            calib_flags,
            fisheye_flags,
            pat,
            DEFAULT_CAMERA_NAME,
            checkerboard_flags=cb_flags,
            max_chessboard_speed=DEFAULT_MAX_CHESSBOARD_SPEED,
            queue_size=DEFAULT_QUEUE_SIZE,
        )

        # State for counting samples
        self._sample_count: int = -1

        # State to avoid re‑running upload
        self._uploaded: bool = False

        # Bridge for cv2 -> ROS conversions
        self._bridge: cv_bridge.CvBridge = cv_bridge.CvBridge()

        # image_transport publisher using "zstd" transport
        self._it_pub: ImageTransport = ImageTransport(
            self.get_name() + "_pub", image_transport="zstd"
        )
        self._calibration_pub: rclpy.publisher.Publisher = self._it_pub.advertise(
            self.resolve_topic_name(CALIBRATION_TOPIC), 1
        )

    def stop(self) -> None:
        self.get_logger().info("Stopping camera calibrator node...")
        self.destroy_node()

    def redraw_monocular(self, drawable: ImageDrawable) -> None:
        """
        Implementation of CalibrationNode.
        """
        sample_count: int = len(self.c.db)
        if sample_count != self._sample_count:
            self._sample_count = sample_count
            self.get_logger().info(
                f"samples={len(self.c.db)} goodenough={self.c.goodenough}"
            )

        # drawable.scrib is BGR8 image slice from Calibrator
        bgr_img: np.ndarray = drawable.scrib

        # Wrap it in a ROS Image message as bgr8
        img_msg: ImageMsg = self._bridge.cv2_to_imgmsg(bgr_img, encoding="bgr8")

        # TODO: Preserve original header
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self._camera_name

        # Publish the image
        self._calibration_pub.publish(img_msg)

        # Check if we have enough images to calibrate
        if self.c.goodenough and not self.c.calibrated:
            self.get_logger().info("Good enough -> calibrating")
            self.c.do_calibration()

        # Check if we have enough images to upload
        if self.c.calibrated and not self._uploaded:
            self.get_logger().info("Calibrated -> uploading CameraInfo")
            self.do_upload()
            self._uploaded = True

    def redraw_stereo(self, drawable: Any) -> None:
        """
        Implementation of CalibrationNode.
        """
        sample_count: int = len(self.c.db)
        if sample_count != self._sample_count:
            self._sample_count = sample_count
            self.get_logger().info(
                f"samples={len(self.c.db)} goodenough={self.c.goodenough}"
            )

        # Grab the left/right BGR images from the drawable
        left_bgr: np.ndarray = drawable.lscrib
        right_bgr: np.ndarray = drawable.rscrib

        # Compute canvas size
        h: int = max(left_bgr.shape[0], right_bgr.shape[0])
        w: int = left_bgr.shape[1] + right_bgr.shape[1]

        # Create an empty BGR canvas and copy both images in
        canvas_bgr: np.ndarray = np.zeros((h, w, 3), dtype=np.uint8)
        canvas_bgr[: left_bgr.shape[0], : left_bgr.shape[1]] = left_bgr
        canvas_bgr[: right_bgr.shape[0], left_bgr.shape[1] :] = right_bgr

        # Wrap the stitched BGR canvas as bgr8
        img_msg: ImageMsg = self._bridge.cv2_to_imgmsg(canvas_bgr, encoding="bgr8")

        # TODO: Preserve original header
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self._camera_name

        # Publish the image
        self._calibration_pub.publish(img_msg)

        # Once we have enough varied samples, do calibration exactly once
        if self.c.goodenough and not self.c.calibrated:
            self.get_logger().info("Good enough -> running stereo calibration")
            self.c.do_calibration()

        # Once calibrated, upload to set_camera_info exactly once
        if self.c.calibrated and not self._uploaded:
            self.get_logger().info("Stereo calibrated -> uploading CameraInfo")
            self.do_upload()
            self._uploaded = True
