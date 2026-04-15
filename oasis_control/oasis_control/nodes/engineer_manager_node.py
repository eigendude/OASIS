################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a Millenium Engineer LEGO model
#

from typing import Optional

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from rclpy.logging import LoggingSeverity
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Header as HeaderMsg

from oasis_control.lego_models.falcon_manager import FalconManager
from oasis_control.managers.sampling_manager import SamplingManager
from oasis_control.mcu.mcu_memory_manager import McuMemoryManager
from oasis_msgs.msg import ConductorState as ConductorStateMsg
from oasis_msgs.msg import EngineerState as EngineerStateMsg


################################################################################
# Hardware configuration
################################################################################


# Sampling interval, in ms
SAMPLING_INTERVAL_MS: int = 100

# Memory reporting interval, in seconds
MEMORY_INTERVAL_SECS: float = 1.0


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = "engineer_manager"

PUBLISH_STATE_PERIOD_SECS = 0.1

# Publisher
PUBLISH_ENGINEER_STATE = "engineer_state"

# Subscribers
CONDUCTOR_STATE_TOPIC = "conductor_state"


################################################################################
# ROS node
################################################################################


class EngineerManagerNode(rclpy.node.Node):
    """
    A ROS node that manages a Millenium Engineer LEGO model.
    """

    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Subsystems
        self._mcu_memory_manager: McuMemoryManager = McuMemoryManager(self)
        self._sampling_manager: SamplingManager = SamplingManager(self)
        self._falcon_manager: FalconManager = FalconManager(self)
        self._falcon_ready: bool = False
        self._pending_falcon_duty_magnitude: Optional[float] = None
        self._falcon_defer_logged: bool = False

        # Reliable QOS profile for state topics
        state_qos_profile: rclpy.qos.QoSProfile = rclpy.qos.QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self._engineer_state_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=EngineerStateMsg,
            topic=PUBLISH_ENGINEER_STATE,
            qos_profile=state_qos_profile,
        )

        # Subscribers
        self._conductor_state_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=ConductorStateMsg,
                topic=CONDUCTOR_STATE_TOPIC,
                callback=self._handle_conductor_state,
                qos_profile=state_qos_profile,
            )
        )

        # Timer parameters
        self._publish_state_timer: Optional[rclpy.node.Timer] = None

    def initialize(self) -> bool:
        self.get_logger().debug("Starting engineer manager configuration")

        # Memory reporting
        if not self._mcu_memory_manager.initialize(MEMORY_INTERVAL_SECS):
            return False

        # Sampling interval
        if not self._sampling_manager.initialize(SAMPLING_INTERVAL_MS):
            return False

        # Initialize LEGO model
        if not self._falcon_manager.initialize():
            return False
        self._falcon_ready = True

        if self._pending_falcon_duty_magnitude is not None:
            pending_duty_magnitude: float = self._pending_falcon_duty_magnitude
            self.get_logger().debug(
                "Applying deferred Falcon conductor duty magnitude="
                f"{pending_duty_magnitude:.3f} after initialization"
            )
            self._falcon_manager.set_thrust_led_effect(pending_duty_magnitude)
            self._pending_falcon_duty_magnitude = None

        # Now that the manager is initialized, start the publishing timer
        self._publish_state_timer = self.create_timer(
            timer_period_sec=PUBLISH_STATE_PERIOD_SECS, callback=self._publish_state
        )

        self.get_logger().info("Engineer manager initialized successfully")

        return True

    def _publish_state(self) -> None:
        header = HeaderMsg()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = NODE_NAME  # TODO

        msg: EngineerStateMsg = EngineerStateMsg()
        msg.header = header
        msg.total_ram = self._mcu_memory_manager.total_ram
        msg.ram_utilization = self._mcu_memory_manager.ram_utilization

        self._engineer_state_pub.publish(msg)

    def _handle_conductor_state(self, msg: ConductorStateMsg) -> None:
        duty_magnitude: float = min(max(abs(float(msg.duty_cycle)), 0.0), 1.0)
        if not self._falcon_ready:
            self._pending_falcon_duty_magnitude = duty_magnitude
            if not self._falcon_defer_logged:
                self.get_logger().debug(
                    "Deferring Falcon conductor duty update until Falcon "
                    "initialization completes"
                )
                self._falcon_defer_logged = True
            return

        self.get_logger().debug(
            "Received conductor_state duty_cycle="
            f"{float(msg.duty_cycle):.3f}, forwarding magnitude={duty_magnitude:.3f}"
        )
        self._falcon_manager.set_thrust_led_effect(duty_magnitude)
