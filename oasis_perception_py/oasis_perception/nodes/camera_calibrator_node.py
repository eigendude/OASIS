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

import functools
import queue
import socket
import threading
from typing import Any
from typing import Callable
from typing import Generic
from typing import Optional
from typing import TypeVar

import cv2
import cv_bridge
import message_filters
import numpy as np
import rclpy.client
import rclpy.node
import rclpy.publisher
import rclpy.qos
from camera_calibration.calibrator import CAMERA_MODEL
from camera_calibration.calibrator import Calibrator
from camera_calibration.calibrator import ChessboardInfo
from camera_calibration.calibrator import ImageDrawable
from camera_calibration.calibrator import Patterns
from camera_calibration.mono_calibrator import MonoCalibrator
from camera_calibration.stereo_calibrator import StereoCalibrator
from image_transport_py import ImageTransport
from message_filters import ApproximateTimeSynchronizer
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.srv import SetCameraInfo as SetCameraInfoSrv

from oasis_msgs.msg import CalibrationStatus as CalibrationStatusMsg


################################################################################
# ROS parameters
################################################################################


# Default node name
NODE_NAME: str = "camera_calibrator"

# Topic names
CALIBRATION_IMAGE_TOPIC: str = "calibration_image"
CALIBRATION_STATUS_TOPIC: str = "calibration_status"

# Parameter names and defaults
CAMERA_MODEL_PARAM: str = "camera_model"
DEFAULT_CAMERA_MODEL_NAME: str = "pinhole"


################################################################################
# Calibration parameters
################################################################################

# Supported camera models exposed through the ROS parameter
CAMERA_MODEL_CHOICES: dict[str, CAMERA_MODEL] = {
    "pinhole": CAMERA_MODEL.PINHOLE,
    "fisheye": CAMERA_MODEL.FISHEYE,
}

# Default camera name
DEFAULT_CAMERA_NAME: str = socket.gethostname().replace("-", "_")

#
# Checkerboard parameters
#

# Default pattern to use for calibration
DEFAULT_PATTERN: str = "chessboard"

# Default size of the calibration board
DEFAULT_SIZE: list[str] = ["8x6"]  # Inner corners of 9x7 board

# Default square size, in meters
DEFAULT_SQUARE: list[float] = [0.025]

#
# Charuco parameters
#

# Default charuco marker size, in meters
DEFAULT_CHARUCO_MARKER_SIZE: list[float] = []

# Default ArUco dictionary names
DEFAULT_ARUCO_DICT: list[str] = []

#
# Synchronization parameters
#

# If > 0.0, use approximate time synchronization with this slop (in seconds)
DEFAULT_APPROXIMATE: float = 0.0

# Queue size for buffering incoming images
DEFAULT_QUEUE_SIZE: int = 1

#
# Pinhole parameters
#

# Set to True to fix the principal point at the image center
DEFAULT_FIX_PRINCIPAL_POINT: bool = False

# Set to True to fix the aspect ratio (fx/fy)
DEFAULT_FIX_ASPECT_RATIO: bool = False

# Set to True to set tangential distortion coefficients to zero
DEFAULT_ZERO_TANGENT_DIST: bool = False

# Number of K coefficients to optimize. 2 = K1 and K2
DEFAULT_K_COEFFICIENTS: int = 2

#
# Fisheye parameters
#

DEFAULT_FISHEYE_RECOMPUTE_EXTRINSICSTS: bool = False

# Set to True to fix skew coefficient (kappa) to zero. ORB_SLAM_OASIS does not
# support non‑zero skew.
DEFAULT_FISHEYE_FIX_SKEW: bool = True

# Set to True to fix the principal point at the image center
DEFAULT_FISHEYE_FIX_PRINCIPAL_POINT: bool = False

# Number of fisheye K coefficients to optimize. 4 = K1, K2, K3 and K4
DEFAULT_FISHEYE_K_COEFFICIENTS: int = 4

# Set to True to check validity of calibration conditions
DEFAULT_FISHEYE_CHECK_CONDITIONS: bool = False

#
# Optimization parameters
#

# Set to True to disable fast check for chessboard corners
DEFAULT_DISABLE_CALIB_CB_FAST_CHECK: bool = False

# Maximum chessboard speed in pixels/second. Set to -1.0 to disable.
DEFAULT_MAX_CHESSBOARD_SPEED: float = -1.0


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
# Utility classes
################################################################################


T = TypeVar("T")


class BufferQueue(queue.Queue[T]):
    """
    Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """

    def put(self, item: T, *args: Any, **kwargs: Any) -> None:
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()


class ConsumerThread(Generic[T], threading.Thread):
    def __init__(self, queue: BufferQueue[T], function: Callable[[T], None]):
        threading.Thread.__init__(self)
        self.queue: BufferQueue[T] = queue
        self.function: Callable[[T], None] = function

    def run(self) -> None:
        while rclpy.ok():
            m: T = self.queue.get()
            try:
                self.function(m)
            except Exception as e:
                # Never let a single bad frame kill the worker thread
                print(f"[camera_calibrator] ConsumerThread error, skipping frame: {e}")
                import traceback

                traceback.print_exc()


################################################################################
# ROS node
################################################################################


class CameraCalibratorNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        # Declare ROS parameters
        self.declare_parameter(CAMERA_MODEL_PARAM, DEFAULT_CAMERA_MODEL_NAME)

        # Resolve camera model parameter
        camera_model_value: str = (
            self.get_parameter(CAMERA_MODEL_PARAM).get_parameter_value().string_value
        )
        camera_model_key: str = camera_model_value.lower()

        if camera_model_key not in CAMERA_MODEL_CHOICES:
            valid_options = ", ".join(sorted(CAMERA_MODEL_CHOICES))
            self.get_logger().warning(
                f"Invalid camera model '{camera_model_value}'. "
                f"Falling back to default '{DEFAULT_CAMERA_MODEL_NAME}'. "
                f"Valid options: {valid_options}"
            )

        self._camera_model: CAMERA_MODEL = CAMERA_MODEL_CHOICES.get(
            camera_model_key, CAMERA_MODEL_CHOICES[DEFAULT_CAMERA_MODEL_NAME]
        )

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
        sync_cls: Callable[..., Any] = (
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
        num_k_coeffs: int = DEFAULT_K_COEFFICIENTS

        if num_k_coeffs > 3:
            calib_flags |= cv2.CALIB_RATIONAL_MODEL
        if num_k_coeffs < 6:
            calib_flags |= cv2.CALIB_FIX_K6
        if num_k_coeffs < 5:
            calib_flags |= cv2.CALIB_FIX_K5
        if num_k_coeffs < 4:
            calib_flags |= cv2.CALIB_FIX_K4
        if num_k_coeffs < 3:
            calib_flags |= cv2.CALIB_FIX_K3
        if num_k_coeffs < 2:
            calib_flags |= cv2.CALIB_FIX_K2
        if num_k_coeffs < 1:
            calib_flags |= cv2.CALIB_FIX_K1

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
        num_fisheye_k_coeffs: int = DEFAULT_FISHEYE_K_COEFFICIENTS
        if num_fisheye_k_coeffs < 4:
            fisheye_flags |= cv2.fisheye.CALIB_FIX_K4
        if num_fisheye_k_coeffs < 3:
            fisheye_flags |= cv2.fisheye.CALIB_FIX_K3
        if num_fisheye_k_coeffs < 2:
            fisheye_flags |= cv2.fisheye.CALIB_FIX_K2
        if num_fisheye_k_coeffs < 1:
            fisheye_flags |= cv2.fisheye.CALIB_FIX_K1

        # Checkerboard fast‑check
        checkerboard_flags: int = (
            0 if DEFAULT_DISABLE_CALIB_CB_FAST_CHECK else cv2.CALIB_CB_FAST_CHECK
        )

        # Pattern enum
        pattern: int = {
            "chessboard": Patterns.Chessboard,
            "circles": Patterns.Circles,
            "acircles": Patterns.ACircles,
            "charuco": Patterns.ChArUco,
        }[DEFAULT_PATTERN]

        # Initialize service clients
        self.set_camera_info_service: rclpy.client.Client = self.create_client(
            SetCameraInfoSrv, "camera/set_camera_info"
        )
        self.set_left_camera_info_service: rclpy.client.Client = self.create_client(
            SetCameraInfoSrv, "left_camera/set_camera_info"
        )
        self.set_right_camera_info_service: rclpy.client.Client = self.create_client(
            SetCameraInfoSrv, "right_camera/set_camera_info"
        )

        # Initialize state
        self._boards: list[ChessboardInfo] = boards
        self._calib_flags: int = calib_flags
        self._fisheye_calib_flags: int = fisheye_flags
        self._checkerboard_flags: int = checkerboard_flags
        self._pattern: int = pattern
        self._camera_name: str = DEFAULT_CAMERA_NAME
        self._max_chessboard_speed: float = DEFAULT_MAX_CHESSBOARD_SPEED

        # Setup message filters
        self._lsub: message_filters.Subscriber = message_filters.Subscriber(
            self, ImageMsg, "left", qos_profile=self.get_topic_qos("left")
        )
        self._rsub: message_filters.Subscriber = message_filters.Subscriber(
            self, ImageMsg, "right", qos_profile=self.get_topic_qos("right")
        )
        self._time_sync: Any = sync_cls([self._lsub, self._rsub], 4)
        self._time_sync.registerCallback(self.queue_stereo)

        self._msub: message_filters.Subscriber = message_filters.Subscriber(
            self, ImageMsg, "image", qos_profile=self.get_topic_qos("image")
        )
        self._msub.registerCallback(self.queue_monocular)

        self.q_mono: BufferQueue[ImageMsg] = BufferQueue(DEFAULT_QUEUE_SIZE)
        self.q_stereo: BufferQueue[tuple[ImageMsg, ImageMsg]] = BufferQueue(
            DEFAULT_QUEUE_SIZE
        )

        self.c: Calibrator | None = None

        self._last_display: Optional[ImageDrawable] = None

        mth: ConsumerThread[ImageMsg] = ConsumerThread(
            self.q_mono, self.handle_monocular
        )
        mth.daemon = True
        mth.start()

        sth: ConsumerThread[tuple[ImageMsg, ImageMsg]] = ConsumerThread(
            self.q_stereo, self.handle_stereo
        )
        sth.daemon = True
        sth.start()

        # State for counting samples
        self._sample_count: int = -1

        self.displaywidth: int = 0

        # State to avoid re‑running upload
        self._uploaded: bool = False

        # Bridge for cv2 -> ROS conversions
        self._bridge: cv_bridge.CvBridge = cv_bridge.CvBridge()

        # Image transport for image publishers and subscribers
        self._it_pub: ImageTransport = ImageTransport(
            self.get_name() + "_image_transport"
        )

        # Publishers
        self._calibration_image_pub: rclpy.publisher.Publisher = self._it_pub.advertise(
            self.resolve_topic_name(CALIBRATION_IMAGE_TOPIC), 1
        )
        self._calibration_status_pub: rclpy.publisher.Publisher = self.create_publisher(
            CalibrationStatusMsg,
            self.resolve_topic_name(CALIBRATION_STATUS_TOPIC),
            1,
        )

    def stop(self) -> None:
        self.get_logger().info("Stopping camera calibrator node...")
        self.destroy_node()

    def queue_monocular(self, msg: ImageMsg) -> None:
        # DDS may loan image buffers that become invalid once the callback
        # returns. Copy the incoming message so the consumer thread always
        # owns a stable buffer.
        self.q_mono.put(self.copy_image_msg(msg))

    def queue_stereo(self, lmsg: ImageMsg, rmsg: ImageMsg) -> None:
        # See queue_monocular() for rationale on copying incoming images.
        self.q_stereo.put((self.copy_image_msg(lmsg), self.copy_image_msg(rmsg)))

    def copy_image_msg(self, msg: ImageMsg) -> ImageMsg:
        """Create a deep copy of an Image message with owned pixel data."""

        msg_copy: ImageMsg = ImageMsg()
        msg_copy.header = msg.header
        msg_copy.height = msg.height
        msg_copy.width = msg.width
        msg_copy.encoding = msg.encoding
        msg_copy.is_bigendian = msg.is_bigendian
        msg_copy.step = msg.step

        # Ensure pixel data is backed by our own buffer instead of a DDS loan.
        msg_copy.data = bytes(msg.data)

        return msg_copy

    def handle_monocular(self, msg: ImageMsg) -> None:
        # Skip obviously invalid/empty images
        if msg.height == 0 or msg.width == 0 or not msg.data:
            self.get_logger().warn(
                "camera_calibrator: received empty monocular image, skipping"
            )
            return

        if self.c is None:
            if self._camera_name:
                self.c = MonoCalibrator(
                    self._boards,
                    self._calib_flags,
                    self._fisheye_calib_flags,
                    self._pattern,
                    name=self._camera_name,
                    checkerboard_flags=self._checkerboard_flags,
                    max_chessboard_speed=self._max_chessboard_speed,
                )
            else:
                self.c = MonoCalibrator(
                    self._boards,
                    self._calib_flags,
                    self._fisheye_calib_flags,
                    self._pattern,
                    checkerboard_flags=self._checkerboard_flags,
                    max_chessboard_speed=self._max_chessboard_speed,
                )

        # Package image_pipeline couldn't set camera model until first image received?
        self.c.set_cammodel(self._camera_model)

        # This should just call the MonoCalibrator
        try:
            drawable: ImageDrawable = self.c.handle_msg(msg)
        except cv_bridge.CvBridgeError as e:
            # Extra introspection for debugging empty cv::Mat / cvtColor failures
            try:
                data_len = len(msg.data)
            except Exception:
                data_len = -1

            self.get_logger().warn(
                "camera_calibrator: CvBridgeError on monocular frame, skipping. "
                f"Error={e}; "
                f"encoding={msg.encoding!r}, "
                f"height={msg.height}, width={msg.width}, "
                f"step={msg.step}, len(data)={data_len}"
            )
            return
        except Exception as e:
            self.get_logger().error(
                "camera_calibrator: unexpected error in handle_monocular, "
                f"skipping frame: {e}"
            )
            return

        self.displaywidth = drawable.scrib.shape[1]
        self.redraw_monocular(drawable)

    def handle_stereo(self, msg: tuple[ImageMsg, ImageMsg]) -> None:
        try:
            lmsg, rmsg = msg
        except Exception:
            self.get_logger().warn(
                "camera_calibrator: stereo callback got malformed message, skipping"
            )
            return
        if (
            lmsg.height == 0
            or lmsg.width == 0
            or not lmsg.data
            or rmsg.height == 0
            or rmsg.width == 0
            or not rmsg.data
        ):
            self.get_logger().warn(
                "camera_calibrator: received empty stereo image(s), skipping"
            )
            return

        if self.c is None:
            if self._camera_name:
                self.c = StereoCalibrator(
                    self._boards,
                    self._calib_flags,
                    self._fisheye_calib_flags,
                    self._pattern,
                    name=self._camera_name,
                    checkerboard_flags=self._checkerboard_flags,
                    max_chessboard_speed=self._max_chessboard_speed,
                )
            else:
                self.c = StereoCalibrator(
                    self._boards,
                    self._calib_flags,
                    self._fisheye_calib_flags,
                    self._pattern,
                    checkerboard_flags=self._checkerboard_flags,
                    max_chessboard_speed=self._max_chessboard_speed,
                )

        # Package image_pipeline couldn't set camera model until first image received?
        self.c.set_cammodel(self._camera_model)

        try:
            drawable = self.c.handle_msg(msg)
        except cv_bridge.CvBridgeError as e:
            self.get_logger().warn(
                f"camera_calibrator: CvBridgeError on stereo frame, skipping: {e}"
            )
            return
        except Exception as e:
            self.get_logger().error(
                f"camera_calibrator: unexpected error in handle_stereo, skipping frame: {e}"
            )
            return
        self.displaywidth = drawable.lscrib.shape[1] + drawable.rscrib.shape[1]
        self.redraw_stereo(drawable)

    def check_set_camera_info(self, response: SetCameraInfoSrv.Response) -> bool:
        if response.success:
            return True

        for i in range(10):
            print("!" * 80)
        print()
        print(f"Attempt to set camera info failed: {response.status_message}")
        print()
        for i in range(10):
            print("!" * 80)
        print()
        self.get_logger().error(
            f"Unable to set camera info for calibration. Failure message: {response.status_message}"
        )
        return False

    def do_upload(self) -> bool:
        assert self.c is not None

        self.c.report()
        print(self.c.ost())
        info: Any = self.c.as_message()

        req: SetCameraInfoSrv.Request = SetCameraInfoSrv.Request()
        rv: bool = True
        if self.c.is_mono:
            req.camera_info = info
            response: SetCameraInfoSrv.Response = self.set_camera_info_service.call(req)
            rv = self.check_set_camera_info(response)
        else:
            req.camera_info = info[0]
            response = self.set_left_camera_info_service.call(req)
            rv = rv and self.check_set_camera_info(response)
            req.camera_info = info[1]
            response = self.set_right_camera_info_service.call(req)
            rv = rv and self.check_set_camera_info(response)
        return rv

    def get_topic_qos(self, topic_name: str) -> rclpy.qos.QoSProfile:
        """
        Given a topic name, get the QoS profile with which it is being
        published.

        Replaces history and depth settings with default values since they
        cannot be retrieved.

        @param topic_name (str) The topic name

        @return QosProfile The QoS profile with which the topic is published.
        If no publishers exist for the given topic, it returns the sensor data
        QoS.
        """
        topic_name = self.resolve_topic_name(topic_name)
        topic_info: list[Any] = self.get_publishers_info_by_topic(topic_name=topic_name)
        if len(topic_info):
            qos_profile: rclpy.qos.QoSProfile = topic_info[0].qos_profile
            qos_profile.history = rclpy.qos.qos_profile_system_default.history
            qos_profile.depth = rclpy.qos.qos_profile_system_default.depth
            return qos_profile
        else:
            self.get_logger().warn(
                f"No publishers available for topic {topic_name}. Using system default QoS for subscriber."
            )
            return rclpy.qos.qos_profile_system_default

    def redraw_monocular(self, drawable: ImageDrawable) -> None:
        assert self.c is not None

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

        # Publish the calibration image
        self._calibration_image_pub.publish(img_msg)

        # Publish the calibration status
        status_msg: CalibrationStatusMsg = CalibrationStatusMsg()
        status_msg.sample_count = sample_count
        status_msg.good_enough = self.c.goodenough
        status_msg.calibrated = self.c.calibrated
        status_msg.uploaded = self._uploaded
        self._calibration_status_pub.publish(status_msg)

        # Check if we have enough images to calibrate
        if self.c.goodenough and not self.c.calibrated:
            self.get_logger().info("Good enough -> calibrating")
            self.c.do_calibration()
            self.get_logger().info("Calibrated successfully")

            # Publish the updated status
            status_msg.calibrated = self.c.calibrated
            self._calibration_status_pub.publish(status_msg)

        # Check if we have enough images to upload
        if self.c.calibrated and not self._uploaded:
            self.get_logger().info("Calibrated -> uploading CameraInfo")
            self.do_upload()
            self._uploaded = True

            # Publish the updated status
            status_msg.uploaded = self._uploaded
            self._calibration_status_pub.publish(status_msg)

    def redraw_stereo(self, drawable: ImageDrawable) -> None:
        assert self.c is not None

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

        # Publish the calibration image
        self._calibration_image_pub.publish(img_msg)

        # Publish the calibration status
        status_msg: CalibrationStatusMsg = CalibrationStatusMsg()
        status_msg.sample_count = sample_count
        status_msg.good_enough = self.c.goodenough
        status_msg.calibrated = self.c.calibrated
        status_msg.uploaded = self._uploaded
        self._calibration_status_pub.publish(status_msg)

        # Once we have enough varied samples, do calibration exactly once
        if self.c.goodenough and not self.c.calibrated:
            self.get_logger().info("Good enough -> running stereo calibration")
            self.c.do_calibration()
            self.get_logger().info("Stereo calibrated successfully")

            # Publish the updated status
            status_msg.calibrated = self.c.calibrated
            self._calibration_status_pub.publish(status_msg)

        # Once calibrated, upload to set_camera_info exactly once
        if self.c.calibrated and not self._uploaded:
            self.get_logger().info("Stereo calibrated -> uploading CameraInfo")
            self.do_upload()
            self._uploaded = True

            # Publish the updated status
            status_msg.uploaded = self._uploaded
            self._calibration_status_pub.publish(status_msg)
