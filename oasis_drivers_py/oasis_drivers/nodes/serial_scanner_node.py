################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from typing import List

import rclpy.node
import rclpy.publisher
import rclpy.qos
from builtin_interfaces.msg import Time as TimeMsg

from oasis_drivers.ros.ros_translator import RosTranslator
from oasis_drivers.serial.serial_scanner import SerialScanner
from oasis_drivers.serial.serial_types import SerialPort
from oasis_msgs.msg import SerialDeviceScan as SerialDeviceScanMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "serial_scanner"

PUBLISH_SERIAL_TOPIC = "serial_ports"

# Rate at which devices are scanned
SCAN_PERIOD_SECS = 10.0


################################################################################
# ROS node
################################################################################


class SerialScannerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._serial_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=SerialDeviceScanMsg,
            topic=PUBLISH_SERIAL_TOPIC,
            qos_profile=qos_profile,
        )

        # Timing parameters
        self._timer: rclpy.node.Timer = self.create_timer(
            timer_period_sec=SCAN_PERIOD_SECS, callback=self._do_scan
        )

        self.get_logger().info("Serial scanner initialized")

    def stop(self) -> None:
        self.get_logger().info("Serial scanner deinitialized")

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        self.destroy_node()

    def _do_scan(self) -> None:
        """
        Scan for serial ports.
        """
        # Perform the scan
        ports: List[SerialPort] = SerialScanner.do_scan()

        # Publish serial port info
        self._publish_msg(ports)

    def _publish_msg(self, ports: List[SerialPort]) -> None:
        # Timestamp for ROS message header
        timestamp_msg: TimeMsg = self.get_clock().now().to_msg()

        msg: SerialDeviceScanMsg = RosTranslator.get_serial_ports_msg(
            ports, timestamp_msg
        )

        self._serial_pub.publish(msg)
