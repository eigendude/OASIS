################################################################################
#
#  Copyright (C) 2021-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch.launch_description import LaunchDescription
from launch_ros.actions import Node

from oasis_control.launch.ahrs_mounting import AhrsMountingConfig


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
    # AHRS
    #

    @staticmethod
    def add_ahrs_node(
        ld: LaunchDescription,
        host_id: str,
        mounting_config: AhrsMountingConfig = AhrsMountingConfig(),
    ) -> None:
        ahrs_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable="ahrs",
            name=f"ahrs_{host_id}",
            output="screen",
            parameters=[
                {
                    "base_frame_id": mounting_config.parent_frame_id,
                    "imu_frame_id": mounting_config.child_frame_id,
                    "mounting_calibration_duration_sec": (
                        mounting_config.calibration_duration_sec
                    ),
                    "mounting_stationary_angular_speed_threshold_rads": (
                        mounting_config.stationary_angular_speed_threshold_rads
                    ),
                    "mounting_min_sample_count": mounting_config.min_sample_count,
                }
            ],
            remappings=[
                ("ahrs/diag", f"{host_id}/ahrs/diag"),
                ("ahrs/imu", f"{host_id}/ahrs/imu"),
                ("ahrs/odom", f"{host_id}/ahrs/odom"),
                ("gravity", f"{host_id}/gravity"),
                ("imu", f"{host_id}/imu"),
            ],
        )
        ld.add_action(ahrs_node)

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
                ("configure_effect", f"{mcu_node}/configure_effect"),
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
                ("set_effect", f"{mcu_node}/set_effect"),
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
        conductor_host: str = "",
    ) -> None:
        remappings: list[tuple[str, str]] = [
            (f"{mcu_node}_state", f"{host_id}/{mcu_node}_state"),
            ("configure_effect", f"{mcu_node}/configure_effect"),
            ("mcu_memory", f"{mcu_node}/mcu_memory"),
            ("pwm_write_cmd", f"{mcu_node}/pwm_write_cmd"),
            ("report_mcu_memory", f"{mcu_node}/report_mcu_memory"),
            ("set_digital_mode", f"{mcu_node}/set_digital_mode"),
            ("set_effect", f"{mcu_node}/set_effect"),
            ("set_sampling_interval", f"{mcu_node}/set_sampling_interval"),
        ]
        if conductor_host:
            remappings.append(("conductor_state", f"{conductor_host}/conductor_state"))

        engineer_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable=f"{mcu_node}_manager",
            name=f"{mcu_node}_manager_{host_id}",
            output="screen",
            remappings=remappings,
        )
        ld.add_action(engineer_node)

    #
    # Speedometer
    #

    @staticmethod
    def add_speedometer_node(ld: LaunchDescription, host_id: str) -> None:
        speedometer_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable="speedometer",
            name=f"speedometer_{host_id}",
            output="screen",
            remappings=[
                ("forward_twist", f"{host_id}/forward_twist"),
                ("imu", f"{host_id}/ahrs/imu"),
                ("zupt", f"{host_id}/zupt"),
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
                ("gravity", f"{host_id}/gravity"),
                ("tilt", f"{host_id}/tilt"),
            ],
        )
        ld.add_action(localization_node)

    @staticmethod
    def add_ahrs_tilt_sensor(ld: LaunchDescription, host_id: str) -> None:
        ahrs_tilt_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable="ahrs_tilt",
            name=f"ahrs_tilt_{host_id}",
            output="screen",
            remappings=[
                ("gravity", f"{host_id}/gravity"),
                ("imu", f"{host_id}/ahrs/imu"),
                ("tilt", f"{host_id}/ahrs/tilt"),
            ],
        )
        ld.add_action(ahrs_tilt_node)

    #
    # ZUPT detector
    #

    @staticmethod
    def add_zupt_detector(ld: LaunchDescription, host_id: str) -> None:
        zupt_detector_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable="zupt_detector",
            name=f"zupt_detector_{host_id}",
            output="screen",
            remappings=[
                ("imu", f"{host_id}/imu"),
                ("zupt_flag", f"{host_id}/zupt_flag"),
                ("zupt", f"{host_id}/zupt"),
            ],
        )
        ld.add_action(zupt_detector_node)
