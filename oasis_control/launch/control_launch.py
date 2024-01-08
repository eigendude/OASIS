################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

import socket

from launch import LaunchDescription
from launch_ros.actions import Node


################################################################################
# System parameters
################################################################################


# Get the hostname
HOSTNAME = socket.gethostname()


################################################################################
# TODO: Hardware configuration
################################################################################


# Machine that broadcasts power on/off commands
POWER_CONTROLLER = "nuc"

# Machine that broadcasts peripheral input
INPUT_PROVIDER = "nuc"


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

PACKAGE_NAME = "oasis_control"


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Machines with power actuation (laptop displays, LED artwork, etc)
    if HOSTNAME in [
        "asus",
        "inspiron",
        "lenovo",
        "netbook",
        "station",
    ]:
        automation_manager_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable="automation_manager",
            name=f"automation_manager_{HOSTNAME}",
            output="screen",
            remappings=[
                ("power_event", f"{POWER_CONTROLLER}/power_event"),
                ("power_control", f"{HOSTNAME}/power_control"),
            ],
        )
        ld.add_action(automation_manager_node)

    # Microcontroller nodes
    if HOSTNAME == "station":
        MCU_NODE = "conductor"
        conductor_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable=f"{MCU_NODE}_manager_telemetrix",
            name=f"{MCU_NODE}_manager_telemetrix_{HOSTNAME}",
            output="screen",
            remappings=[
                (f"{MCU_NODE}_state", f"{HOSTNAME}/{MCU_NODE}_state"),
                ("analog_reading", f"{MCU_NODE}/analog_reading"),
                ("capture_input", f"{INPUT_PROVIDER}/capture_input"),
                ("cpu_fan_speed", f"{MCU_NODE}/cpu_fan_speed"),
                ("cpu_fan_write", f"{MCU_NODE}/cpu_fan_write"),
                ("digital_reading", f"{MCU_NODE}/digital_reading"),
                ("digital_write", f"{MCU_NODE}/digital_write"),
                ("input", f"{INPUT_PROVIDER}/input"),
                ("mcu_memory", f"{MCU_NODE}/mcu_memory"),
                ("mcu_string", f"{MCU_NODE}/mcu_string"),
                ("peripherals", f"{INPUT_PROVIDER}/peripherals"),
                ("power_control", f"{HOSTNAME}/power_control"),
                ("pwm_write", f"{MCU_NODE}/pwm_write"),
                ("report_mcu_memory", f"{MCU_NODE}/report_mcu_memory"),
                ("set_analog_mode", f"{MCU_NODE}/set_analog_mode"),
                (
                    "set_cpu_fan_sampling_interval",
                    f"{MCU_NODE}/set_cpu_fan_sampling_interval",
                ),
                ("set_digital_mode", f"{MCU_NODE}/set_digital_mode"),
                ("set_sampling_interval", f"{MCU_NODE}/set_sampling_interval"),
            ],
        )
        ld.add_action(conductor_node)
    elif HOSTNAME == "jetson":
        MCU_NODE = "engine"
        engine_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable=f"{MCU_NODE}_manager",
            name=f"{MCU_NODE}_manager_{HOSTNAME}",
            output="screen",
            remappings=[
                (f"{MCU_NODE}_state", f"{HOSTNAME}/{MCU_NODE}_state"),
                ("analog_reading", f"{MCU_NODE}/analog_reading"),
                ("mcu_memory", f"{MCU_NODE}/mcu_memory"),
                ("mcu_string", f"{MCU_NODE}/mcu_string"),
                ("report_mcu_memory", f"{MCU_NODE}/report_mcu_memory"),
                ("set_analog_mode", f"{MCU_NODE}/set_analog_mode"),
                ("set_sampling_interval", f"{MCU_NODE}/set_sampling_interval"),
            ],
        )
        ld.add_action(engine_node)
    elif HOSTNAME == "substation":
        MCU_NODE = "lab"
        CPU_FAN_HOST = "conductor"
        lab_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable=f"{MCU_NODE}_manager",
            name=f"{MCU_NODE}_manager_{HOSTNAME}",
            output="screen",
            remappings=[
                (f"{MCU_NODE}_state", f"{HOSTNAME}/{MCU_NODE}_state"),
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
                ("air_quality", f"{MCU_NODE}/air_quality"),
                ("analog_reading", f"{MCU_NODE}/analog_reading"),
                ("digital_write", f"{MCU_NODE}/digital_write"),
                ("i2c_begin", f"{MCU_NODE}/i2c_begin"),
                ("i2c_end", f"{MCU_NODE}/i2c_end"),
                ("i2c_imu", f"{MCU_NODE}/i2c_imu"),
                ("mcu_memory", f"{MCU_NODE}/mcu_memory"),
                ("report_mcu_memory", f"{MCU_NODE}/report_mcu_memory"),
                ("set_analog_mode", f"{MCU_NODE}/set_analog_mode"),
                ("set_digital_mode", f"{MCU_NODE}/set_digital_mode"),
                ("set_sampling_interval", f"{MCU_NODE}/set_sampling_interval"),
            ],
        )
        ld.add_action(lab_node)

    return ld
