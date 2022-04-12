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


HOSTNAME = socket.gethostname()


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

CPP_PACKAGE_NAME = "oasis_drivers_cpp"
PYTHON_PACKAGE_NAME = "oasis_drivers_py"


################################################################################
# Hardware/video parameters
################################################################################


# Hardware options
ENABLE_CEC = False
ENABLE_DISPLAY = False
ENABLE_FIRMATA = False
ENABLE_KINECT_V2 = False
ENABLE_VIDEO = False

# Video parameters
VIDEO_DEVICE = "/dev/video0"
IMAGE_SIZE = [640, 480]

# Firmata parameters
AVR_COM_PORT = "/dev/ttyACM0"

# TODO: Hardware configuration
if HOSTNAME == "asus":
    ENABLE_DISPLAY = True
elif HOSTNAME == "cinder":
    ENABLE_KINECT_V2 = True
elif HOSTNAME == "inspiron":
    ENABLE_DISPLAY = True
elif HOSTNAME == "lenovo":
    ENABLE_DISPLAY = True
    ENABLE_VIDEO = True
elif HOSTNAME == "netbook":
    ENABLE_DISPLAY = True
    ENABLE_VIDEO = True
elif HOSTNAME == "nuc":
    ENABLE_CEC = True


print(f"Launching on {HOSTNAME}")


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    if ENABLE_CEC:
        cec_server_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="cec_server",
            name=f"cec_server_{HOSTNAME}",
            output="screen",
            remappings=[
                ("power_control", f"{HOSTNAME}/power_control"),
                ("power_event", f"{HOSTNAME}/power_event"),
            ],
        )
        ld.add_action(cec_server_node)

    if ENABLE_DISPLAY:
        display_manager_node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="display_manager",
            name=f"display_manager_{HOSTNAME}",
            output="screen",
            remappings=[
                ("power_control", f"{HOSTNAME}/power_control"),
            ],
        )
        ld.add_action(display_manager_node)

    serial_port_node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="serial_scanner",
        name=f"serial_scanner_{HOSTNAME}",
        output="screen",
        remappings=[
            ("serial_ports", f"{HOSTNAME}/serial_ports"),
        ],
    )
    ld.add_action(serial_port_node)

    system_monitor_node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="system_monitor",
        name=f"system_monitor_{HOSTNAME}",
        output="screen",
        remappings=[
            ("system_telemetry", f"{HOSTNAME}/system_telemetry"),
        ],
    )
    ld.add_action(system_monitor_node)

    if ENABLE_VIDEO:
        v4l2_node = Node(
            namespace=ROS_NAMESPACE,
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name=f"v4l2_camera_{HOSTNAME}",
            output="screen",
            parameters=[
                {
                    "image_size": IMAGE_SIZE,
                    "video_device": VIDEO_DEVICE,
                },
            ],
            remappings=[
                ("camera_info", f"{HOSTNAME}/camera_info"),
                ("image_raw", f"{HOSTNAME}/image_raw"),
                ("image_raw/compressed", f"{HOSTNAME}/image_raw/compressed"),
                ("image_raw/compressedDepth", f"{HOSTNAME}/image_raw/compressedDepth"),
                ("image_raw/theora", f"{HOSTNAME}/image_raw/theora"),
            ],
        )
        ld.add_action(v4l2_node)

    if ENABLE_KINECT_V2:
        kinect_v2_node = Node(
            namespace=ROS_NAMESPACE,
            package="kinect2_bridge",
            executable="kinect2_bridge",
            name=f"kinect2_bridge_{HOSTNAME}",
            output="screen",
        )
        ld.add_action(kinect_v2_node)

    # Microcontroller interfaces
    if HOSTNAME == "station":
        MCU_NODE = "conductor"
        conductor_bridge_node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="firmata_bridge",
            name=f"{MCU_NODE}_bridge_{HOSTNAME}",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "com_port": AVR_COM_PORT,
                },
            ],
            remappings=[
                ("analog_read", f"{MCU_NODE}/analog_read"),
                ("analog_reading", f"{MCU_NODE}/analog_reading"),
                ("cpu_fan_speed", f"{MCU_NODE}/cpu_fan_speed"),
                ("digital_read", f"{MCU_NODE}/digital_read"),
                ("digital_reading", f"{MCU_NODE}/digital_reading"),
                ("digital_write", f"{MCU_NODE}/digital_write"),
                ("mcu_memory", f"{MCU_NODE}/mcu_memory"),
                ("pwm_write", f"{MCU_NODE}/pwm_write"),
                ("report_mcu_memory", f"{MCU_NODE}/report_mcu_memory"),
                ("servo_write", f"{MCU_NODE}/servo_write"),
                ("set_analog_mode", f"{MCU_NODE}/set_analog_mode"),
                ("set_digital_mode", f"{MCU_NODE}/set_digital_mode"),
                ("string_message", f"{MCU_NODE}/string_message"),
            ],
        )
        ld.add_action(conductor_bridge_node)
    elif HOSTNAME == "cinder":
        MCU_NODE = "leonardo"
        leonardo_bridge_node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="firmata_bridge",
            name=f"{MCU_NODE}_bridge_{HOSTNAME}",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "com_port": AVR_COM_PORT,
                },
            ],
            remappings=[
                ("analog_read", f"{MCU_NODE}/analog_read"),
                ("analog_reading", f"{MCU_NODE}/analog_reading"),
                ("cpu_fan_speed", f"{MCU_NODE}/cpu_fan_speed"),
                ("digital_read", f"{MCU_NODE}/digital_read"),
                ("digital_reading", f"{MCU_NODE}/digital_reading"),
                ("digital_write", f"{MCU_NODE}/digital_write"),
                ("mcu_memory", f"{MCU_NODE}/mcu_memory"),
                ("pwm_write", f"{MCU_NODE}/pwm_write"),
                ("report_mcu_memory", f"{MCU_NODE}/report_mcu_memory"),
                ("servo_write", f"{MCU_NODE}/servo_write"),
                ("set_analog_mode", f"{MCU_NODE}/set_analog_mode"),
                ("set_digital_mode", f"{MCU_NODE}/set_digital_mode"),
                ("string_message", f"{MCU_NODE}/string_message"),
            ],
        )
        ld.add_action(leonardo_bridge_node)

    return ld
