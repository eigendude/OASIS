################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

import socket

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


################################################################################
# System parameters
################################################################################


HOSTNAME = socket.gethostname().replace("-", "_")


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
ENABLE_CAMERA = False

# Video parameters
VIDEO_DEVICE = "/dev/video0"
IMAGE_SIZE = [640, 480]

# Firmata parameters
AVR_COM_PORT = "/dev/ttyACM0"

# TODO: Hardware configuration
if HOSTNAME == "asus":
    ENABLE_DISPLAY = True
    ENABLE_VIDEO = False
elif HOSTNAME == "cinder":
    ENABLE_KINECT_V2 = True
elif HOSTNAME == "lenovo":
    ENABLE_DISPLAY = True
    ENABLE_VIDEO = False
elif HOSTNAME == "station":
    ENABLE_CAMERA = True


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
        display_server_node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="display_server",
            name=f"display_server_{HOSTNAME}",
            output="screen",
            remappings=[
                ("power_control", f"{HOSTNAME}/power_control"),
            ],
        )
        ld.add_action(display_server_node)

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
                ("image_raw/zstd", f"{HOSTNAME}/image_raw/zstd"),
            ],
        )
        ld.add_action(v4l2_node)

    if ENABLE_CAMERA:
        camera_node = Node(
            namespace=ROS_NAMESPACE,
            package="camera_ros",
            executable="camera_node",
            name=f"camera_ros_{HOSTNAME}",
            output="screen",
            parameters=[
                {
                    "format": "RGB888",
                    "width": IMAGE_SIZE[0],
                    "height": IMAGE_SIZE[1],
                    "sensor_mode": "3280:2464",  # V2 camera full sensor resolution
                },
            ],
            remappings=[
                (f"camera_ros_{HOSTNAME}/camera_info", f"{HOSTNAME}/camera_info"),
                (f"camera_ros_{HOSTNAME}/image_raw", f"{HOSTNAME}/image_raw"),
                (
                    f"camera_ros_{HOSTNAME}/image_raw/compressed",
                    f"{HOSTNAME}/image_raw/compressed",
                ),
            ],
        )
        ld.add_action(camera_node)

    if ENABLE_KINECT_V2:
        # TODO: For now use a separate node for the main Kinect V2 bridge
        kinect2_bridge_node = Node(
            namespace=ROS_NAMESPACE,
            package="kinect2_bridge",
            executable="kinect2_bridge",
            name=f"kinect2_bridge_{HOSTNAME}",
            output="screen",
            parameters=[
                {"reg_method": "opencl"},
            ],
        )
        ld.add_action(kinect2_bridge_node)

        kinect2_container = ComposableNodeContainer(
            namespace=ROS_NAMESPACE,
            name=f"kinect2_container_{HOSTNAME}",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                # TODO: Composable nodes seem to break when not using the modern image_transport API
                # ComposableNode(
                #     package="kinect2_bridge",
                #     plugin="kinect2_bridge::Kinect2BridgeComponent",
                #     name=f"kinect2_bridge_{HOSTNAME}",
                # ),
                ComposableNode(
                    package="kinect2_bridge",
                    plugin="kinect2_bridge::Kinect2DownscalerComponent",
                    name=f"kinect2_downscaler_{HOSTNAME}",
                ),
            ],
        )
        ld.add_action(kinect2_container)

    # Template for Firmata bridges
    def get_firmata_bridge(hostname: str, mcu_name: str) -> Node:
        return Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="firmata_bridge",
            name=f"{mcu_name}_bridge_{hostname}",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "com_port": AVR_COM_PORT,
                },
            ],
            remappings=[
                ("analog_read", f"{mcu_name}/analog_read"),
                ("analog_reading", f"{mcu_name}/analog_reading"),
                ("cpu_fan_speed", f"{mcu_name}/cpu_fan_speed"),
                ("digital_read", f"{mcu_name}/digital_read"),
                ("digital_reading", f"{mcu_name}/digital_reading"),
                ("digital_write", f"{mcu_name}/digital_write"),
                ("mcu_memory", f"{mcu_name}/mcu_memory"),
                ("mcu_string", f"{mcu_name}/mcu_string"),
                ("pwm_write", f"{mcu_name}/pwm_write"),
                ("report_mcu_memory", f"{mcu_name}/report_mcu_memory"),
                ("servo_write", f"{mcu_name}/servo_write"),
                ("set_analog_mode", f"{mcu_name}/set_analog_mode"),
                ("set_digital_mode", f"{mcu_name}/set_digital_mode"),
                ("set_sampling_interval", f"{mcu_name}/set_sampling_interval"),
            ],
        )

    # Template for Telemetrix bridges
    def get_telemetrix_bridge(hostname: str, mcu_name: str) -> Node:
        return Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="telemetrix_bridge",
            name=f"{mcu_name}_bridge_{hostname}",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "com_port": AVR_COM_PORT,
                },
            ],
            remappings=[
                ("air_quality", f"{mcu_name}/air_quality"),
                ("analog_read", f"{mcu_name}/analog_read"),
                ("analog_reading", f"{mcu_name}/analog_reading"),
                ("cpu_fan_speed", f"{mcu_name}/cpu_fan_speed"),
                ("cpu_fan_write", f"{mcu_name}/cpu_fan_write"),
                ("digital_read", f"{mcu_name}/digital_read"),
                ("digital_reading", f"{mcu_name}/digital_reading"),
                ("digital_write", f"{mcu_name}/digital_write"),
                ("i2c_begin", f"{mcu_name}/i2c_begin"),
                ("i2c_end", f"{mcu_name}/i2c_end"),
                ("i2c_imu", f"{mcu_name}/i2c_imu"),
                ("mcu_memory", f"{mcu_name}/mcu_memory"),
                ("mcu_string", f"{mcu_name}/mcu_string"),
                ("pwm_write", f"{mcu_name}/pwm_write"),
                ("report_mcu_memory", f"{mcu_name}/report_mcu_memory"),
                ("servo_write", f"{mcu_name}/servo_write"),
                ("set_analog_mode", f"{mcu_name}/set_analog_mode"),
                (
                    "set_cpu_fan_sampling_interval",
                    f"{mcu_name}/set_cpu_fan_sampling_interval",
                ),
                ("set_digital_mode", f"{mcu_name}/set_digital_mode"),
                ("set_sampling_interval", f"{mcu_name}/set_sampling_interval"),
            ],
        )

    # Microcontroller interfaces
    if HOSTNAME == "station":
        mcu_node = "conductor"
        conductor_bridge_node: Node = get_telemetrix_bridge(HOSTNAME, mcu_node)
        ld.add_action(conductor_bridge_node)
    elif HOSTNAME == "jetson":
        mcu_node = "engine"
        engine_bridge_node: Node = get_telemetrix_bridge(HOSTNAME, mcu_node)
        ld.add_action(engine_bridge_node)
    elif HOSTNAME == "substation":
        mcu_node = "lab"
        lab_bridge_node: Node = get_telemetrix_bridge(HOSTNAME, mcu_node)
        ld.add_action(lab_bridge_node)

    return ld
