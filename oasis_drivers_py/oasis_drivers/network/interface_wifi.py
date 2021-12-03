################################################################################
#
#  Copyright (C) 2016-2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from Interface import Interface
from Interface import InterfaceType


"""
from oasis_msgs.srv import CheckIsWireless as CheckIsWirelessSvc
from oasis_msgs.srv import EndScan as EndScanSvc
from oasis_msgs.srv import StartScan as StartScanSvc
"""


WIFI_SERVICE = "check_is_wireless"
START_SCAN_SERVICE = "start_wifi_scan"
END_SCAN_SERVICE = "end_wifi_scan"


class InterfaceWiFi(Interface):
    def __init__(self, name, logger):
        super().__init__(name)

        self._logger = logger

    def type(self) -> InterfaceType:
        return InterfaceType.WIFI

    @staticmethod
    def check_is_wireless(interface_name: str) -> bool:
        result = False

        """
        #rospy.wait_for_service(WIFI_SERVICE)

        serviceProxy = rospy.ServiceProxy(WIFI_SERVICE, CheckIsWirelessSvc)

        try:
            result = serviceProxy(interfaceName).is_wireless
        except rospy.ServiceException as ex:
            rospy.logerr('Service did not process request: ' + str(ex))
        """

        return result

    def start_passive_scan(self) -> bool:
        result = False

        """
        #rospy.wait_for_service(TART_SCAN_SERVICE)

        serviceProxy = rospy.ServiceProxy(START_SCAN_SERVICE, StartScanSvc)

        try:
            serviceProxy(self.name(), True, [], [])
            result = True
        except rospy.ServiceException as ex:
            rospy.logerr('Service did not process request: ' + str(ex))
        """

        return result

    def end_scan(self) -> None:
        """
        #rospy.wait_for_service(END_SCAN_SERVICE)

        serviceProxy = rospy.ServiceProxy(END_SCAN_SERVICE, EndScanSvc)

        try:
            serviceProxy(self.name())
        except rospy.ServiceException as ex:
            rospy.logerr('Service did not process request: ' + str(ex))
        """
        pass
