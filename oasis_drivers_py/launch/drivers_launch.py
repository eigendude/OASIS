################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


################################################################################
# System parameters
################################################################################


HOSTNAME: str = socket.gethostname().replace("-", "_")


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE: str = "oasis"

CPP_PACKAGE_NAME: str = "oasis_drivers_cpp"
PYTHON_PACKAGE_NAME: str = "oasis_drivers_py"
HASS_PACKAGE_NAME: str = "oasis_hass"


################################################################################
# Hardware/video parameters
################################################################################


# Hardware options
ENABLE_CEC = False
ENABLE_DISPLAY = False
ENABLE_HOME_ASSISTANT = False
ENABLE_KINECT_V2 = False
ENABLE_VIDEO = False
ENABLE_CAMERA = False

# Video parameters
VIDEO_DEVICE = "/dev/video0"
IMAGE_SIZE = [640, 480]

# Firmata parameters
AVR_COM_PORT = "/dev/ttyACM0"

# TODO: Hardware configuration
if HOSTNAME == "bar":
    ENABLE_DISPLAY = True
    ENABLE_VIDEO = True
elif HOSTNAME == "cinder":  # Perception server
    ENABLE_KINECT_V2 = True
if HOSTNAME == "door":
    ENABLE_DISPLAY = True
    ENABLE_CAMERA = True
elif HOSTNAME == "homeassistant":
    ENABLE_HOME_ASSISTANT = True
elif HOSTNAME == "kitchen":
    ENABLE_DISPLAY = True
    ENABLE_VIDEO = True
elif HOSTNAME == "nuc":  # Hallway HUD
    ENABLE_DISPLAY = True
elif HOSTNAME == "station":  # LEGO Train Station
    ENABLE_CAMERA = True

# The base name for the Kinect V2 bridge
KINECT_V2_BASE_NAME: str = "hallway"


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
                ("set_display", f"{HOSTNAME}/set_display"),
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
        video_container: ComposableNodeContainer = ComposableNodeContainer(
            namespace=ROS_NAMESPACE,
            name=f"video_container_{HOSTNAME}",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="v4l2_camera",
                    plugin="v4l2_camera::V4L2Camera",
                    name=f"v4l2_camera_{HOSTNAME}",
                    parameters=[
                        {
                            "image_size": IMAGE_SIZE,
                            "video_device": VIDEO_DEVICE,
                        },
                    ],
                    remappings=[
                        ("camera_info", f"{HOSTNAME}/camera_info"),
                        ("image_raw", f"{HOSTNAME}/image_raw"),
                    ],
                ),
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name=f"rectify_node_{HOSTNAME}",
                    remappings=[
                        ("image", f"{HOSTNAME}/image_raw"),
                        ("camera_info", f"{HOSTNAME}/camera_info"),
                        ("image_rect", f"{HOSTNAME}/image_rect"),
                    ],
                    parameters=[
                        {"image_transport": "compressed"},
                        {"interpolation": 1},  # Linear
                    ],
                ),
            ],
        )
        ld.add_action(video_container)

    if ENABLE_CAMERA:
        camera_container: ComposableNodeContainer = ComposableNodeContainer(
            namespace=ROS_NAMESPACE,
            package="rclcpp_components",
            executable="component_container_mt",
            name=f"camera_container_{HOSTNAME}",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="camera_ros",
                    plugin="camera::CameraNode",
                    name=f"camera_ros_{HOSTNAME}",
                    parameters=[
                        {
                            "format": "RGB888",
                            "width": IMAGE_SIZE[0],
                            "height": IMAGE_SIZE[1],
                            "sensor_mode": "3280:2464",  # V2 camera full sensor resolution
                        },
                    ],
                    remappings=[
                        (
                            f"camera_ros_{HOSTNAME}/camera_info",
                            f"{HOSTNAME}/camera_info",
                        ),
                        (f"camera_ros_{HOSTNAME}/image_raw", f"{HOSTNAME}/image_raw"),
                        (
                            f"camera_ros_{HOSTNAME}/image_raw/compressed",
                            f"{HOSTNAME}/image_raw/compressed",
                        ),
                    ],
                ),
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name=f"rectify_node_{HOSTNAME}",
                    remappings=[
                        ("image", f"{HOSTNAME}/image_raw"),
                        ("camera_info", f"{HOSTNAME}/camera_info"),
                        ("image_rect", f"{HOSTNAME}/image_rect"),
                    ],
                    parameters=[
                        {"image_transport": "compressed"},
                        {"interpolation": 1},  # Linear
                    ],
                ),
            ],
        )
        ld.add_action(camera_container)

    if ENABLE_KINECT_V2:
        # TODO: For now use a separate node for the main Kinect V2 bridge.
        # Running as a component breaks the legacy image transport code.
        kinect2_bridge_node = Node(
            namespace=ROS_NAMESPACE,
            package="kinect2_bridge",
            executable="kinect2_bridge",
            name=f"kinect2_bridge_{HOSTNAME}",
            output="screen",
            parameters=[
                {"base_name": KINECT_V2_BASE_NAME},
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
                    namespace=ROS_NAMESPACE,
                    package="kinect2_bridge",
                    plugin="kinect2_bridge::Kinect2DownscalerComponent",
                    name=f"kinect2_downscaler_{HOSTNAME}",
                    # TODO: Modify downscaler to not hardcode the base name
                    remappings=[
                        (
                            "kinect2/qhd/image_color",
                            f"{KINECT_V2_BASE_NAME}/qhd/image_color",
                        ),
                        (
                            "kinect2/sd/image_color",
                            f"{KINECT_V2_BASE_NAME}/sd/image_color",
                        ),
                    ],
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

    if ENABLE_HOME_ASSISTANT:
        hass_bridge_node = Node(
            namespace=ROS_NAMESPACE,
            package=HASS_PACKAGE_NAME,
            executable="hass_bridge",
            name=f"hass_bridge_{HOSTNAME}",
            output="screen",
            remappings=[
                ("plug", f"{HOSTNAME}/plug"),
                ("rgb", f"{HOSTNAME}/rgb"),
                ("set_plug", f"{HOSTNAME}/set_plug"),
                ("set_rgb", f"{HOSTNAME}/set_rgb"),
            ],
        )
        ld.add_action(hass_bridge_node)

        # Start the generic MQTT -> ROS bridge
        params_file: str = os.path.join(
            get_package_share_directory(HASS_PACKAGE_NAME),
            "mqtt_client",
            "mqtt_client_params.yaml",
        )
        mqtt_client_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package="mqtt_client",
            executable="mqtt_client",
            name=f"mqtt_client_{HOSTNAME}",
            output="screen",
            parameters=[params_file],
            remappings=[
                ("statestream", f"{HOSTNAME}/statestream"),
            ],
        )
        ld.add_action(mqtt_client_node)

        # Also run the WoL server
        wol_server_node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="wol_server",
            name=f"wol_server_{HOSTNAME}",
            output="screen",
            remappings=[
                ("get_mac_address", f"{HOSTNAME}/get_mac_address"),
                ("wol", f"{HOSTNAME}/wol"),
            ],
        )
        ld.add_action(wol_server_node)

    # UPS server
    ups_server_node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="ups_server",
        name=f"ups_server_{HOSTNAME}",
        output="screen",
        remappings=[
            ("ups_command", f"{HOSTNAME}/ups_command"),
            ("ups_status", f"{HOSTNAME}/ups_status"),
        ],
    )
    ld.add_action(ups_server_node)

    return ld
