################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE: str = "oasis"

CONTROL_PACKAGE_NAME: str = "oasis_control"


################################################################################
# Node descriptions
################################################################################


class ControlDescriptions:
    #
    # EKF localizer
    #

    @staticmethod
    def add_ekf_localizer(
        ld: LaunchDescription, host_id: str, apriltag_resolution: str
    ) -> None:
        ekf_localizer_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable="ekf_localizer",
            name=f"ekf_localizer_{host_id}",
            output="screen",
            remappings=[
                ("apriltags", f"{host_id}/{apriltag_resolution}/apriltags"),
                ("camera_info", f"{host_id}/{apriltag_resolution}/camera_info"),
                ("ekf/updates/apriltags", f"{host_id}/ekf/updates/apriltags"),
                ("ekf/updates/imu", f"{host_id}/ekf/updates/imu"),
                ("ekf/updates/mag", f"{host_id}/ekf/updates/mag"),
                ("imu_calibration", f"{host_id}/imu_calibration"),
                ("imu_raw", f"{host_id}/imu_raw"),
                ("magnetic_field", f"{host_id}/magnetic_field"),
                ("odom", f"{host_id}/odom"),
                ("world_odom", f"{host_id}/world_odom"),
            ],
        )
        ld.add_action(ekf_localizer_node)

    #
    # Home manager
    #

    @staticmethod
    def add_home_manager(
        ld: LaunchDescription,
        smart_display_zones: list[str],
        smart_display_plug_id: str,
        camera_zones: list[str],
        home_assistant_id: str,
    ) -> None:
        home_manager_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable="home_manager",
            name="home_manager",
            output="screen",
            arguments=["--ros-args", "--log-level", "home_manager:=info"],
            parameters=[
                {
                    "smart_display_zones": smart_display_zones,
                    "smart_display_plug_id": smart_display_plug_id,
                }
            ],
            remappings=[
                *[
                    (f"camera_scene_{zone_id}", f"{zone_id}/camera_scene")
                    for zone_id in camera_zones
                ],
                ("plug", f"{home_assistant_id}/plug"),
                ("rgb", f"{home_assistant_id}/rgb"),
                *[
                    (f"set_display_{zone_id}", f"{zone_id}/set_display")
                    for zone_id in smart_display_zones
                ],
                ("set_plug", f"{home_assistant_id}/set_plug"),
                ("set_rgb", f"{home_assistant_id}/set_rgb"),
            ],
        )
        ld.add_action(home_manager_node)

    #
    # IMU fuser
    #

    @staticmethod
    def add_imu_fuser(ld: LaunchDescription, host_id: str) -> None:
        imu_fuser_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable="imu_fuser",
            name=f"imu_fuser_{host_id}",
            output="screen",
            remappings=[
                ("imu", f"{host_id}/imu"),
                ("imu_fused", f"{host_id}/imu_fused"),
                ("magnetic_field", f"{host_id}/magnetic_field"),
            ],
        )
        ld.add_action(imu_fuser_node)

    #
    # Microcontroller nodes
    #

    @staticmethod
    def add_mcu_manager_telemetrix(
        ld: LaunchDescription,
        host_id: str,
        mcu_node: str,
        wol_server_id: str,
        input_provider: str,
        calibration_resolution: str,
    ) -> None:
        conductor_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable=f"{mcu_node}_manager_telemetrix",
            name=f"{mcu_node}_manager_telemetrix_{host_id}",
            output="screen",
            remappings=[
                (f"{mcu_node}_state", f"{host_id}/{mcu_node}_state"),
                ("analog_reading", f"{mcu_node}/analog_reading"),
                (
                    "calibration_status",
                    f"{mcu_node}/{calibration_resolution}/calibration_status",
                ),
                ("capture_input", f"{input_provider}/capture_input"),
                ("cpu_fan_speed", f"{mcu_node}/cpu_fan_speed"),
                ("cpu_fan_write", f"{mcu_node}/cpu_fan_write"),
                ("digital_reading", f"{mcu_node}/digital_reading"),
                ("digital_write_cmd", f"{mcu_node}/digital_write_cmd"),
                ("get_mac_address", f"{wol_server_id}/get_mac_address"),
                ("input", f"{input_provider}/input"),
                ("mcu_memory", f"{mcu_node}/mcu_memory"),
                ("mcu_string", f"{mcu_node}/mcu_string"),
                ("peripherals", f"{input_provider}/peripherals"),
                ("power_control", f"{host_id}/power_control"),
                ("pwm_write_cmd", f"{mcu_node}/pwm_write_cmd"),
                ("report_mcu_memory", f"{mcu_node}/report_mcu_memory"),
                ("set_analog_mode", f"{mcu_node}/set_analog_mode"),
                (
                    "set_cpu_fan_sampling_interval",
                    f"{mcu_node}/set_cpu_fan_sampling_interval",
                ),
                ("set_digital_mode", f"{mcu_node}/set_digital_mode"),
                ("set_sampling_interval", f"{mcu_node}/set_sampling_interval"),
                ("wol", f"{host_id}/wol"),
            ],
        )
        ld.add_action(conductor_node)

    @staticmethod
    def add_mcu_manager(
        ld: LaunchDescription,
        host_id: str,
        mcu_node: str,
    ) -> None:
        engine_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable=f"{mcu_node}_manager",
            name=f"{mcu_node}_manager_{host_id}",
            output="screen",
            remappings=[
                (f"{mcu_node}_state", f"{host_id}/{mcu_node}_state"),
                ("analog_reading", f"{mcu_node}/analog_reading"),
                ("mcu_memory", f"{mcu_node}/mcu_memory"),
                ("mcu_string", f"{mcu_node}/mcu_string"),
                ("report_mcu_memory", f"{mcu_node}/report_mcu_memory"),
                ("set_analog_mode", f"{mcu_node}/set_analog_mode"),
                ("set_sampling_interval", f"{mcu_node}/set_sampling_interval"),
            ],
        )
        ld.add_action(engine_node)

    @staticmethod
    def add_mcu_manager_with_pwm(
        ld: LaunchDescription,
        host_id: str,
        mcu_node: str,
    ) -> None:
        engineer_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable=f"{mcu_node}_manager",
            name=f"{mcu_node}_manager_{host_id}",
            output="screen",
            remappings=[
                (f"{mcu_node}_state", f"{host_id}/{mcu_node}_state"),
                ("mcu_memory", f"{mcu_node}/mcu_memory"),
                ("pwm_write_cmd", f"{mcu_node}/pwm_write_cmd"),
                ("report_mcu_memory", f"{mcu_node}/report_mcu_memory"),
                ("set_digital_mode", f"{mcu_node}/set_digital_mode"),
                ("set_sampling_interval", f"{mcu_node}/set_sampling_interval"),
            ],
        )
        ld.add_action(engineer_node)

    #
    # Speedometer
    #

    @staticmethod
    def add_speedometer(ld: LaunchDescription, host_id: str) -> None:
        speedometer_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable="speedometer",
            name=f"speedometer_{host_id}",
            output="screen",
            remappings=[
                ("imu_fused", f"{host_id}/imu_fused"),
                ("twist", f"{host_id}/twist"),
            ],
        )
        ld.add_action(speedometer_node)

    #
    # Tilt sensor
    #

    @staticmethod
    def add_tilt_sensor(ld: LaunchDescription, host_id: str) -> None:
        localization_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable="tilt_sensor",
            name=f"tilt_sensor_{host_id}",
            output="screen",
            remappings=[
                ("imu", f"{host_id}/imu"),
                ("imu_calibration", f"{host_id}/imu_calibration"),
                ("tilt", f"{host_id}/tilt"),
            ],
        )
        ld.add_action(localization_node)
