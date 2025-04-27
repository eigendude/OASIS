################################################################################
#
#  Copyright (C) 2022-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a mcu that controls an MPU-6050 IMU sensor over I2C
#

import rclpy.node
import rclpy.qos
import rclpy.subscription
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.managers.i2c_device_manager import I2CDeviceManager
from oasis_msgs.msg import I2CDevice as I2CDeviceMsg
from oasis_msgs.msg import I2CDeviceType as I2CDeviceTypeMsg
from oasis_msgs.msg import I2CImu as I2CImuMsg


################################################################################
# ROS parameters
################################################################################


# Subscribers
SUBSCRIBE_IMU = "i2c_imu"


################################################################################
# ROS node
################################################################################


class MPU6050Manager(I2CDeviceManager):
    """
    A manager for an MPU-6050 IMU sensor connected to a microcontroller
    """

    def __init__(self, node: rclpy.node.Node, i2c_port: int, i2c_address: int) -> None:
        """
        Initialize resources.
        """
        super().__init__(node, i2c_port, i2c_address)

        # Initialize ROS parameters
        self._node: rclpy.node.Node = node
        self._logger = node.get_logger()

        # Initialize hardware state
        self._ax: float = 0.0
        self._ay: float = 0.0
        self._az: float = 0.0
        self._gx: float = 0.0
        self._gy: float = 0.0
        self._gz: float = 0.0

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribers
        self._air_quality_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=ImuMsg,
                topic=SUBSCRIBE_IMU,
                callback=self._on_imu,
                qos_profile=qos_profile,
            )
        )

    def initialize(self) -> bool:
        if not super().initialize():
            return False

        self._logger.debug("Starting MPU-6050 configuration")

        # Set up I2C device
        self._logger.debug(
            f"Enabling MPU-6050 on I2C{self.i2c_port + 1} at address {hex(self.i2c_address)}"
        )
        if not self.i2c_begin_device(
            I2CDeviceTypeMsg.MPU6050, self.i2c_port, self.i2c_address
        ):
            return False

        self._logger.info("MPU-6050 manager initialized successfully")

        return True

    @property
    def ax(self) -> float:
        return self._ax

    @property
    def ay(self) -> float:
        return self._ay

    @property
    def az(self) -> float:
        return self._az

    @property
    def gx(self) -> float:
        return self._gx

    @property
    def gy(self) -> float:
        return self._gy

    @property
    def gz(self) -> float:
        return self._gz

    def _on_imu(self, i2c_imu_msg: I2CImuMsg) -> None:
        # Translate parameters
        i2c_device: I2CDeviceMsg = i2c_imu_msg.i2c_device
        imu_msg: ImuMsg = i2c_imu_msg.imu

        if (
            i2c_device.i2c_port == self.i2c_port
            and i2c_device.i2c_address == self.i2c_address
        ):
            # Record state
            self._ax = imu_msg.linear_acceleration.x
            self._ay = imu_msg.linear_acceleration.y
            self._az = imu_msg.linear_acceleration.z
            self._gx = imu_msg.angular_velocity.x
            self._gy = imu_msg.angular_velocity.y
            self._gz = imu_msg.angular_velocity.z
