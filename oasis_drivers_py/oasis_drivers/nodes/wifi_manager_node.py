################################################################################
#
#  Copyright (C) 2016-2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import rclpy.node
import rclpy.publisher
import rclpy.qos

from oasis_drivers.network.wifi_manager import WiFiManager
from oasis_msgs.msg import WiFiScanData as WiFiScanDataMsg
from oasis_msgs.srv import CheckIsWireless as CheckIsWirelessSvc
from oasis_msgs.srv import EndScan as EndScanSvc
from oasis_msgs.srv import StartScan as StartScanSvc


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "wifi_manager"

WIFI_STATUS_TOPIC = "wifi_status"

CHECK_WIRELESS_SERVICE = "check_is_wireless"
START_SCAN_SERVICE = "start_wifi_scan"
END_SCAN_SERVICE = "end_wifi_scan"

################################################################################
# Timing parameters
################################################################################

# Wait at least 1s between scans
SCAN_PERIOD_SECS = 1

################################################################################
# ROS node
################################################################################


class WiFiManagerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Create WiFi manager
        self._wifi_manager = WiFiManager(self.get_logger())
        # TODO: Better lifecycle management
        if not self._wifi_manager.initialize():
            raise Exception("Failed to initialize WiFi manager")

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._wifi_status_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=WiFiScanDataMsg,
            topic=WIFI_STATUS_TOPIC,
            qos_profile=qos_profile,
        )

        # Services
        self._check_wireless_service: rclpy.service.Service = self.create_service(
            srv_type=CheckIsWirelessSvc,
            srv_name=CHECK_WIRELESS_SERVICE,
            callback=self._handle_check_wireless,
        )
        self._start_scan_service: rclpy.service.Service = self.create_service(
            srv_type=StartScanSvc,
            srv_name=START_SCAN_SERVICE,
            callback=self._handle_start_scan,
        )
        self._end_scan_service: rclpy.service.Service = self.create_service(
            srv_type=EndScanSvc,
            srv_name=END_SCAN_SERVICE,
            callback=self._handle_end_scan,
        )

        # Timing parameters
        self._timer: rclpy.node.Timer = self.create_timer(
            timer_period_sec=SCAN_PERIOD_SECS, callback=self._do_scan
        )

        self.get_logger().info("Wifi manager initialized")

    def _handle_check_wireless(self, request, response):
        device: str = request.device
        response.is_wireless = self._wifi_manager.check_is_wireless(device)
        return response

    def _handle_start_scan(self, request, response) -> None:
        self._wifi_manager.start_scan(request.interface)
        return response

    def _handle_end_scan(self, request, response) -> None:
        self._wifi_manager.start_scan(request.interface)
        return response

    def _do_scan(self) -> None:
        """
        Scan for WiFi access points.
        """
        for interface in self._wifi_manager.get_interfaces():
            msg = interface.scan()
            if msg:
                self._wifi_status_pub.publish(msg)
