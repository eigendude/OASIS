################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node


ROS_NAMESPACE = "oasis"

CPP_PACKAGE_NAME = "oasis_drivers_cpp"
PYTHON_PACKAGE_NAME = "oasis_drivers_py"

# TODO: Hardware parameters
MACHINE = "lenovo"

# TODO: Hardware/video parameters
ENABLE_CEC = False
ENABLE_DISPLAY = True
ENABLE_VIDEO = True
VIDEO_DEVICE = "/dev/video0"
IMAGE_SIZE = [640, 480]
ENABLE_KINECT_V2 = False
ENABLE_FIRMATA = False

# TODO: Temporary
print(f"Launching on {MACHINE}")


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    if ENABLE_CEC:
        cec_server_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="cec_server",
            output="screen",
            remappings=[
                ("power_control", f"{MACHINE}/power_control"),
                ("power_event", f"{MACHINE}/power_event"),
            ],
        )
        ld.add_action(cec_server_node)

    device_manager_node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="device_manager",
        output="screen",
    )
    ld.add_action(device_manager_node)

    if ENABLE_DISPLAY:
        display_manager_node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="display_manager",
            output="screen",
            remappings=[
                ("power_control", f"{MACHINE}/power_control"),
            ],
        )
        ld.add_action(display_manager_node)

    led_server_node = Node(
        namespace=ROS_NAMESPACE,
        package=CPP_PACKAGE_NAME,
        executable="led_server",
        output="screen",
    )
    ld.add_action(led_server_node)

    serial_port_node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="serial_scanner",
        output="screen",
        remappings=[
            ("serial_ports", f"{MACHINE}/serial_ports"),
        ],
    )
    ld.add_action(serial_port_node)

    system_monitor_node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="system_monitor",
        output="screen",
        remappings=[
            ("system_telemetry", f"{MACHINE}/system_telemetry"),
        ],
    )
    ld.add_action(system_monitor_node)

    if ENABLE_VIDEO:
        v4l2_node = Node(
            namespace=ROS_NAMESPACE,
            package="v4l2_camera",
            executable="v4l2_camera_node",
            output="screen",
            parameters=[
                {
                    "image_size": IMAGE_SIZE,
                    "video_device": VIDEO_DEVICE,
                },
            ],
            remappings=[
                ("camera_info", f"{MACHINE}/camera_info"),
                ("image_raw", f"{MACHINE}/image_raw"),
                ("image_raw/compressed", f"{MACHINE}/image_raw/compressed"),
                ("image_raw/compressedDepth", f"{MACHINE}/image_raw/compressedDepth"),
                ("image_raw/theora", f"{MACHINE}/image_raw/theora"),
            ],
        )
        ld.add_action(v4l2_node)

    if ENABLE_KINECT_V2:
        kinect_v2_node = Node(
            namespace=ROS_NAMESPACE,
            package="kinect2_bridge",
            executable="kinect2_bridge",
            output="screen",
        )
        ld.add_action(kinect_v2_node)

    if ENABLE_FIRMATA:
        firmata_bridge_node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="firmata_bridge",
            output="screen",
            emulate_tty=True,
            remappings=[
                ("analog_read", f"{MACHINE}/analog_read"),
                ("analog_reading", f"{MACHINE}/analog_reading"),
                ("digital_read", f"{MACHINE}/digital_read"),
                ("digital_reading", f"{MACHINE}/digital_reading"),
                ("digital_write", f"{MACHINE}/digital_write"),
                ("mcu_memory", f"{MACHINE}/mcu_memory"),
                ("pwm_write", f"{MACHINE}/pwm_write"),
                ("report_mcu_memory", f"{MACHINE}/report_mcu_memory"),
                ("servo_write", f"{MACHINE}/servo_write"),
                ("set_analog_mode", f"{MACHINE}/set_analog_mode"),
                ("set_digital_mode", f"{MACHINE}/set_digital_mode"),
                ("string_message", f"{MACHINE}/string_message"),
            ],
        )
        ld.add_action(firmata_bridge_node)

    return ld
