################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  This file is derived from the image_pipeline package under the BSD license.
#  Copyright (C) 2009, Willow Garage, Inc.
#
#  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import cv2
import numpy as np
from camera_calibration.calibrator import CalibrationException
from camera_calibration.mono_calibrator import MonoCalibrator as BaseMonoCalibrator
from sensor_msgs.msg import Image as ImageMsg


class MonoCalibrator(BaseMonoCalibrator):
    """
    Mono camera calibrator with OASIS customizations.

    We override mkgray() to avoid cv_bridge and operate directly on the
    raw sensor_msgs/Image buffer for common encodings. For any encoding
    we don't explicitly handle, we fall back to the base implementation.
    """

    def mkgray(self, msg: ImageMsg) -> np.ndarray:
        """
        Convert a sensor_msgs/Image into an 8-bit 1-channel grayscale OpenCV image.

        Fast-path encodings:
          - "mono8" (already grayscale)
          - "bgr8"  (converted via cv2.COLOR_BGR2GRAY)
          - "rgb8"  (converted via cv2.COLOR_RGB2GRAY)

        For all other encodings, defer to BaseMonoCalibrator.mkgray().
        """
        encoding: str = msg.encoding
        height: int = int(msg.height)
        width: int = int(msg.width)
        step: int = int(msg.step)
        data_bytes: bytes = bytes(msg.data)

        if height <= 0 or width <= 0 or len(data_bytes) == 0:
            raise CalibrationException(
                "MonoCalibrator.mkgray: empty image "
                f"(encoding={encoding!r}, height={height}, width={width}, "
                f"len(data)={len(data_bytes)})"
            )

        # Interpret raw buffer as uint8
        buf: np.ndarray = np.frombuffer(data_bytes, dtype=np.uint8)
        expected_min_size: int = height * step
        if buf.size < expected_min_size:
            raise CalibrationException(
                "MonoCalibrator.mkgray: buffer too small for image "
                f"(encoding={encoding!r}, height={height}, step={step}, "
                f"buf.size={buf.size}, expected>={expected_min_size})"
            )

        gray: np.ndarray

        # Fast-path: mono8
        if encoding == "mono8":
            try:
                reshaped: np.ndarray = buf.reshape(height, step)
            except ValueError as exc:
                raise CalibrationException(
                    f"MonoCalibrator.mkgray: reshape failed for mono8 image: {exc}"
                )
            row_slice: np.ndarray = reshaped[:, :width]
            gray = np.ascontiguousarray(row_slice, dtype=np.uint8)

        # Fast-path: bgr8 / rgb8
        elif encoding in ("bgr8", "rgb8"):
            channels: int = 3
            row_bytes: int = width * channels
            if step < row_bytes:
                raise CalibrationException(
                    "MonoCalibrator.mkgray: step too small for color image "
                    f"(encoding={encoding!r}, height={height}, width={width}, "
                    f"step={step}, required>={row_bytes})"
                )

            try:
                reshaped_color_rows: np.ndarray = buf.reshape(height, step)
            except ValueError as exc:
                raise CalibrationException(
                    f"MonoCalibrator.mkgray: reshape failed for {encoding!r} image: {exc}"
                )

            color_rows: np.ndarray = reshaped_color_rows[:, :row_bytes]

            try:
                color: np.ndarray = color_rows.reshape(height, width, channels)
            except ValueError as exc:
                raise CalibrationException(
                    f"MonoCalibrator.mkgray: color reshape failed for {encoding!r}: {exc}"
                )

            if encoding == "bgr8":
                gray_color: np.ndarray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            else:
                gray_color = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY)

            gray = np.ascontiguousarray(gray_color, dtype=np.uint8)

        # Fallback: let the base class (cv_bridge-based) handle all other encodings
        if gray is None:
            try:
                gray = super().mkgray(msg)
            except Exception as exc:
                raise CalibrationException(
                    "MonoCalibrator.mkgray: fallback to BaseMonoCalibrator.mkgray "
                    f"failed for encoding={encoding!r}: {exc}"
                )

        return gray
