################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node

from oasis_hass.utils.smarthome_config import SmarthomeConfig


################################################################################
# Smarthome parameters
################################################################################


CONFIG: SmarthomeConfig = SmarthomeConfig()

# Get the hostname
HOSTNAME: str = CONFIG.HOSTNAME

# Host aliases
HOST_ID: str = CONFIG.HOST_ID

# Zone configuration
ZONE_ID: str = CONFIG.ZONE_ID

# Zones with a smart display that can be controlled
SMART_DISPLAY_ZONES: list[str] = CONFIG.SMART_DISPLAY_ZONES

# Zones with a camera feed
CAMERA_ZONES: list[str] = CONFIG.CAMERA_ZONES

# The host and zone IDs used for Home Assistant
HOME_ASSISTANT_ID: str = CONFIG.HOME_ASSISTANT_ID

# Machine that broadcasts peripheral input
INPUT_PROVIDER: str = "nuc"  # TODO

print(f"Launching on {HOSTNAME} in zone {ZONE_ID}")


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE: str = "oasis"

CONTROL_PACKAGE_NAME: str = "oasis_control"


################################################################################
# Node definitions
################################################################################


#
# Home manager
#


def add_home_manager(ld: LaunchDescription) -> None:
    home_manager_node: Node = Node(
        namespace=ROS_NAMESPACE,
        package=CONTROL_PACKAGE_NAME,
        executable="home_manager",
        name="home_manager",
        output="screen",
        remappings=[
            *[
                (f"camera_scene_{zone_id}", f"{zone_id}/camera_scene")
                for zone_id in CAMERA_ZONES
            ],
            ("plug", f"{HOME_ASSISTANT_ID}/plug"),
            ("rgb", f"{HOME_ASSISTANT_ID}/rgb"),
            *[
                (f"set_display_{zone_id}", f"{zone_id}/set_display")
                for zone_id in SMART_DISPLAY_ZONES
            ],
            ("set_plug", f"{HOME_ASSISTANT_ID}/set_plug"),
            ("set_rgb", f"{HOME_ASSISTANT_ID}/set_rgb"),
        ],
    )
    ld.add_action(home_manager_node)


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    if HOST_ID == HOME_ASSISTANT_ID:
        add_home_manager(ld)

    # Microcontroller nodes
    if HOST_ID == "station":
        mcu_node = "conductor"
        conductor_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable=f"{mcu_node}_manager_telemetrix",
            name=f"{mcu_node}_manager_telemetrix_{HOST_ID}",
            output="screen",
            remappings=[
                (f"{mcu_node}_state", f"{HOST_ID}/{mcu_node}_state"),
                ("analog_reading", f"{mcu_node}/analog_reading"),
                ("capture_input", f"{INPUT_PROVIDER}/capture_input"),
                ("cpu_fan_speed", f"{mcu_node}/cpu_fan_speed"),
                ("cpu_fan_write", f"{mcu_node}/cpu_fan_write"),
                ("digital_reading", f"{mcu_node}/digital_reading"),
                ("digital_write", f"{mcu_node}/digital_write"),
                ("input", f"{INPUT_PROVIDER}/input"),
                ("mcu_memory", f"{mcu_node}/mcu_memory"),
                ("mcu_string", f"{mcu_node}/mcu_string"),
                ("peripherals", f"{INPUT_PROVIDER}/peripherals"),
                ("power_control", f"{HOST_ID}/power_control"),
                ("pwm_write", f"{mcu_node}/pwm_write"),
                ("report_mcu_memory", f"{mcu_node}/report_mcu_memory"),
                ("set_analog_mode", f"{mcu_node}/set_analog_mode"),
                (
                    "set_cpu_fan_sampling_interval",
                    f"{mcu_node}/set_cpu_fan_sampling_interval",
                ),
                ("set_digital_mode", f"{mcu_node}/set_digital_mode"),
                ("set_sampling_interval", f"{mcu_node}/set_sampling_interval"),
            ],
        )
        ld.add_action(conductor_node)
    elif HOST_ID == "jetson":
        mcu_node = "engine"
        engine_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable=f"{mcu_node}_manager",
            name=f"{mcu_node}_manager_{HOST_ID}",
            output="screen",
            remappings=[
                (f"{mcu_node}_state", f"{HOST_ID}/{mcu_node}_state"),
                ("analog_reading", f"{mcu_node}/analog_reading"),
                ("mcu_memory", f"{mcu_node}/mcu_memory"),
                ("mcu_string", f"{mcu_node}/mcu_string"),
                ("report_mcu_memory", f"{mcu_node}/report_mcu_memory"),
                ("set_analog_mode", f"{mcu_node}/set_analog_mode"),
                ("set_sampling_interval", f"{mcu_node}/set_sampling_interval"),
            ],
        )
        ld.add_action(engine_node)
    elif HOST_ID == "substation":
        mcu_node = "lab"
        CPU_FAN_HOST: str = "conductor"
        lab_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=CONTROL_PACKAGE_NAME,
            executable=f"{mcu_node}_manager",
            name=f"{mcu_node}_manager_{HOST_ID}",
            output="screen",
            remappings=[
                (f"{mcu_node}_state", f"{HOST_ID}/{mcu_node}_state"),
                (f"cpu_fan_speed_{CPU_FAN_HOST}", f"{CPU_FAN_HOST}/cpu_fan_speed"),
                (f"cpu_fan_write_{CPU_FAN_HOST}", f"{CPU_FAN_HOST}/cpu_fan_write"),
                (f"digital_write_{CPU_FAN_HOST}", f"{CPU_FAN_HOST}/digital_write"),
                (
                    f"set_cpu_fan_sampling_interval_{CPU_FAN_HOST}",
                    f"{CPU_FAN_HOST}/set_cpu_fan_sampling_interval",
                ),
                (
                    f"set_digital_mode_{CPU_FAN_HOST}",
                    f"{CPU_FAN_HOST}/set_digital_mode",
                ),
                ("air_quality", f"{mcu_node}/air_quality"),
                ("analog_reading", f"{mcu_node}/analog_reading"),
                ("digital_write", f"{mcu_node}/digital_write"),
                ("i2c_begin", f"{mcu_node}/i2c_begin"),
                ("i2c_end", f"{mcu_node}/i2c_end"),
                ("i2c_imu", f"{mcu_node}/i2c_imu"),
                ("mcu_memory", f"{mcu_node}/mcu_memory"),
                ("report_mcu_memory", f"{mcu_node}/report_mcu_memory"),
                ("set_analog_mode", f"{mcu_node}/set_analog_mode"),
                ("set_digital_mode", f"{mcu_node}/set_digital_mode"),
                ("set_sampling_interval", f"{mcu_node}/set_sampling_interval"),
            ],
        )
        ld.add_action(lab_node)

    return ld
