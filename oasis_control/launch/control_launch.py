################################################################################
#
#  Copyright (C) 2021 Garrett Brown
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
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

PACKAGE_NAME = "oasis_control"


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    if HOSTNAME in [
        "asus",
        "cinder",
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
                # TODO: Hardware configuration
                ("power_event", "nuc/power_event"),
                ("power_control", f"{HOSTNAME}/power_control"),
            ],
        )
        ld.add_action(automation_manager_node)

    if HOSTNAME in [
        "nuc",
        "station",
    ]:
        station_manager_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable="station_manager",
            name=f"station_manager_{HOSTNAME}",
            output="screen",
            remappings=[
                ("analog_reading", f"{HOSTNAME}/analog_reading"),
                ("capture_input", "cinder/capture_input"),
                ("cpu_fan_speed", f"{HOSTNAME}/cpu_fan_speed"),
                ("digital_reading", f"{HOSTNAME}/digital_reading"),
                ("digital_write", f"{HOSTNAME}/digital_write"),
                ("input", "cinder/input"),
                ("mcu_memory", f"{HOSTNAME}/mcu_memory"),
                ("peripherals", "cinder/peripherals"),
                ("power_control", f"{HOSTNAME}/power_control"),
                ("pwm_write", f"{HOSTNAME}/pwm_write"),
                ("report_mcu_memory", f"{HOSTNAME}/report_mcu_memory"),
                ("set_analog_mode", f"{HOSTNAME}/set_analog_mode"),
                ("set_digital_mode", f"{HOSTNAME}/set_digital_mode"),
                ("station_state", f"{HOSTNAME}/station_state"),
            ],
        )
        ld.add_action(station_manager_node)

    if HOSTNAME == "cinder":
        leonardo_manager_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable="leonardo_manager",
            name=f"leonardo_manager_{HOSTNAME}",
            output="screen",
            remappings=[
                ("analog_reading", f"{HOSTNAME}/analog_reading"),
                ("digital_reading", f"{HOSTNAME}/digital_reading"),
                ("digital_write", f"{HOSTNAME}/digital_write"),
                ("mcu_memory", f"{HOSTNAME}/mcu_memory"),
                ("pwm_write", f"{HOSTNAME}/pwm_write"),
                ("report_mcu_memory", f"{HOSTNAME}/report_mcu_memory"),
                ("set_analog_mode", f"{HOSTNAME}/set_analog_mode"),
                ("set_digital_mode", f"{HOSTNAME}/set_digital_mode"),
                ("leonardo_state", f"{HOSTNAME}/leonardo_state"),
            ],
        )
        ld.add_action(leonardo_manager_node)

    return ld
