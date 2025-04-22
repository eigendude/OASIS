################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import threading
from typing import Optional

import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.service

from oasis_drivers.ups.ups_server import UpsServer
from oasis_msgs.msg import UPSStatus as UPSStatusMsg
from oasis_msgs.srv import UPSCommand as UPSCommandSvc


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "ups_server"

UPS_STATUS_PUBLISHER = "ups_status"

UPS_COMMAND_SERVICE = "ups_command"


################################################################################
# Timing parameters
################################################################################


# The UPS server will poll the UPS every second when connected, and every 10s
# when disconnected. This is a compromise between responsiveness and CPU usage.
UPS_POLLING_INTERVAL_CONNECTED = 1.0  # seconds
UPS_POLLING_INTERVAL_DISCONNECTED = 10.0  # seconds


################################################################################
# ROS node
################################################################################


class UpsServerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # ROS publishers
        self._ups_status_publisher: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=UPSStatusMsg,
            topic=UPS_STATUS_PUBLISHER,
            qos_profile=rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        # ROS services
        self._ups_control_service: rclpy.service.Service = self.create_service(
            srv_type=UPSCommandSvc,
            srv_name=UPS_COMMAND_SERVICE,
            callback=self._handle_ups_command,
        )

        # Threading parameters
        self._exit_event = threading.Event()
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)

        self.get_logger().info("UPS server initialized")

        # Start the monitoring thread
        self._thread.start()

    def stop(self) -> None:
        self._exit_event.set()
        self._thread.join()

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        self.destroy_node()

        self.get_logger().info("UPS server deinitialized")

    def _monitor_loop(self) -> None:
        # Loop state
        ups_connected: bool = False
        error_logged: bool = False
        msg: Optional[UPSStatusMsg] = None

        while rclpy.ok():
            # Enter idle mode if the UPS is not connected
            if not ups_connected:
                # Idle mode: check every 10s for a UPS
                msg = UpsServer.read_status(self.get_logger())

                # If we get a valid message, the UPS is connected
                if msg is not None:
                    # Update state
                    ups_connected = True
                    error_logged = False

                    # Log success message
                    self.get_logger().info(
                        f"UPS connected, entering active mode at {UPS_POLLING_INTERVAL_CONNECTED}s intervals"
                    )

                    # Publish message
                    self._ups_status_publisher.publish(msg)

                    # Fall through into active mode
                    pass
                else:
                    if not error_logged:
                        # Update state
                        error_logged = True

                        # Log error message
                        self.get_logger().error(
                            f"No UPS detected, retrying every {UPS_POLLING_INTERVAL_DISCONNECTED}s"
                        )

                    # Sleep before retrying in idle mode
                    if self._exit_event.wait(timeout=UPS_POLLING_INTERVAL_DISCONNECTED):
                        # Exit signal received, exit from idle mode
                        break

                    # No UPS detected, re-enter idle mode
                    continue

            # Active mode: UPS is connected, poll and publish regularly
            msg = UpsServer.read_status(self.get_logger())

            # If we get a invalid message, the UPS is disconnected
            if msg is None:
                # Update state
                ups_connected = False

                # Log error message
                self.get_logger().error("UPS disconnected, returning to idle mode")

                # Loop back into idle mode
                continue

            # Publish message
            self._ups_status_publisher.publish(msg)

            # Wait up to 1s for exit signal, then loop in active mode again
            if self._exit_event.wait(timeout=UPS_POLLING_INTERVAL_CONNECTED):
                # Exit signal received, exit from active mode
                break

    def _handle_ups_command(
        self, request: UPSCommandSvc.Request, response: UPSCommandSvc.Response
    ) -> UPSCommandSvc.Response:
        success: bool = UpsServer.send_command(
            request.command, request.delay, self.get_logger()
        )

        # Log result
        if success:
            self.get_logger().info(
                f"UPS command '{request.command}' executed successfully"
            )
        else:
            self.get_logger().error(f"UPS command '{request.command}' failed")

        # We can enhance this to include success/failure fields if needed
        return response
