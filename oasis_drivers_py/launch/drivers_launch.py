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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

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

# The host and zone IDs used for Home Assistant
HOME_ASSISTANT_ID: str = CONFIG.HOME_ASSISTANT_ID

# The zone ID used for the Kinect V2 camera
KINECT_V2_ZONE_ID: str = CONFIG.KINECT_V2_ZONE_ID

# Zones with a smart display that can be controlled
SMART_DISPLAY_ZONES: list[str] = CONFIG.SMART_DISPLAY_ZONES

# Path to the MQTT parameters file for Home Assistant
MQTT_PARAMS_FILE: str = CONFIG.MQTT_PARAMS_FILE

print(f"Launching on {HOSTNAME} in zone {ZONE_ID}")


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE: str = "oasis"

CPP_PACKAGE_NAME: str = "oasis_drivers_cpp"
PYTHON_PACKAGE_NAME: str = "oasis_drivers_py"
HASS_PACKAGE_NAME: str = CONFIG.HASS_PACKAGE_NAME


################################################################################
# Hardware/video parameters
################################################################################


# Video parameters
VIDEO_DEVICE: str = "/dev/video0"
IMAGE_SIZE: list[int] = [1280, 720]

# Firmata parameters
AVR_COM_PORT: str = "/dev/ttyACM0"


################################################################################
# Node definitions
################################################################################


#
# Display server
#


def add_display_server(ld: LaunchDescription, zone_id: str) -> None:
    display_server_node: Node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="display_server",
        name=f"display_server_{zone_id}",
        output="screen",
        remappings=[
            # Topics
            ("set_display", f"{zone_id}/set_display"),
        ],
    )
    ld.add_action(display_server_node)


#
# Home Assistant nodes
#


def add_home_assistant(ld: LaunchDescription) -> None:
    hass_mqtt_bridge_node: Node = Node(
        namespace=ROS_NAMESPACE,
        package=HASS_PACKAGE_NAME,
        executable="hass_mqtt_bridge",
        name="hass_mqtt_bridge",
        output="screen",
        remappings=[
            # Topics
            ("plug", f"{HOME_ASSISTANT_ID}/plug"),
            ("rgb", f"{HOME_ASSISTANT_ID}/rgb"),
            # Services
            ("set_plug", f"{HOME_ASSISTANT_ID}/set_plug"),
            ("set_rgb", f"{HOME_ASSISTANT_ID}/set_rgb"),
        ],
    )
    ld.add_action(hass_mqtt_bridge_node)

    # Start the generic MQTT -> ROS bridge
    # TODO: Disabled to save resources until mqtt_client is implemented
    """
    mqtt_client_node: Node = Node(
        namespace=ROS_NAMESPACE,
        package="mqtt_client",
        executable="mqtt_client",
        name="mqtt_client",
        output="screen",
        parameters=[MQTT_PARAMS_FILE],
        remappings=[
            # Topics
            ("statestream", f"{HOME_ASSISTANT_ID}/statestream"),
        ],
    )
    ld.add_action(mqtt_client_node)
    """

    # Also run the WoL server
    wol_server_node: Node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="wol_server",
        name="wol_server",
        output="screen",
        remappings=[
            # Services
            ("get_mac_address", f"{HOME_ASSISTANT_ID}/get_mac_address"),
            ("wol", f"{HOME_ASSISTANT_ID}/wol"),
        ],
    )
    ld.add_action(wol_server_node)


#
# Kinect V2 camera
#


def add_kinect_v2(ld: LaunchDescription, zone_id: str) -> None:
    # TODO: For now use a separate node for the main Kinect V2 bridge.
    # Running as a component breaks the legacy image transport code.
    kinect2_bridge_node: Node = Node(
        namespace=ROS_NAMESPACE,
        package="kinect2_bridge",
        executable="kinect2_bridge",
        name=f"kinect2_bridge_{zone_id}",
        output="screen",
        parameters=[
            {"base_name": zone_id, "reg_method": "opencl"},
        ],
    )
    ld.add_action(kinect2_bridge_node)

    kinect2_container: ComposableNodeContainer = ComposableNodeContainer(
        namespace=ROS_NAMESPACE,
        name=f"kinect2_container_{zone_id}",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            # TODO: Composable nodes seem to break when not using the modern image_transport API
            # ComposableNode(
            #     package="kinect2_bridge",
            #     plugin="kinect2_bridge::Kinect2BridgeComponent",
            #     name=f"kinect2_bridge_{zone_id}",
            # ),
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package="kinect2_bridge",
                plugin="kinect2_bridge::Kinect2DownscalerComponent",
                name=f"kinect2_downscaler_{zone_id}",
                parameters=[
                    {
                        "base_name": zone_id,
                    },
                ],
            ),
        ],
    )
    ld.add_action(kinect2_container)


#
# ROS2 (libcamera) camera
#


def add_ros2_camera(ld: LaunchDescription, zone_id: str) -> None:
    camera_container: ComposableNodeContainer = ComposableNodeContainer(
        namespace=ROS_NAMESPACE,
        package="rclcpp_components",
        executable="component_container_mt",
        name=f"camera_container_{zone_id}",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package="camera_ros",
                plugin="camera::CameraNode",
                name=f"camera_ros_{zone_id}",
                parameters=[
                    {
                        "format": "RGB888",
                        "width": IMAGE_SIZE[0],
                        "height": IMAGE_SIZE[1],
                        "sensor_mode": "3280:2464",  # V2 camera full sensor resolution
                    },
                ],
                remappings=[
                    # Topics
                    (
                        f"camera_ros_{zone_id}/camera_info",
                        f"{zone_id}/camera_info",
                    ),
                    (
                        f"camera_ros_{zone_id}/image_raw",
                        f"{zone_id}/image_raw",
                    ),
                    # TODO: Need to remap compressed image topic?
                    (
                        f"camera_ros_{zone_id}/image_raw/compressed",
                        f"{zone_id}/image_raw/compressed",
                    ),
                ],
            ),
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name=f"rectify_node_{zone_id}",
                parameters=[
                    {"image_transport": "compressed", "interpolation": 1},  # Linear
                ],
                remappings=[
                    # Topics
                    ("camera_info", f"{zone_id}/camera_info"),
                    ("image_rect", f"{zone_id}/image_rect"),
                    ("image", f"{zone_id}/image_raw"),
                ],
            ),
        ],
    )
    ld.add_action(camera_container)


#
# Serial port scanner
#


def add_serial_port_scanner(ld: LaunchDescription, host_id: str) -> None:
    serial_port_node: Node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="serial_scanner",
        name=f"serial_scanner_{host_id}",
        output="screen",
        remappings=[
            # Topics
            ("serial_ports", f"{host_id}/serial_ports"),
        ],
    )
    ld.add_action(serial_port_node)


#
# System monitor
#


def add_system_monitor(ld: LaunchDescription, host_id: str) -> None:
    system_monitor_node: Node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="system_monitor",
        name=f"system_monitor_{host_id}",
        output="screen",
        remappings=[
            # Topics
            ("system_telemetry", f"{host_id}/system_telemetry"),
        ],
    )
    ld.add_action(system_monitor_node)


#
# V4L2 camera
#


def add_v4l2_camera(ld: LaunchDescription, zone_id: str) -> None:
    video_container: ComposableNodeContainer = ComposableNodeContainer(
        namespace=ROS_NAMESPACE,
        name=f"video_container_{zone_id}",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package="v4l2_camera",
                plugin="v4l2_camera::V4L2Camera",
                name=f"v4l2_camera_{zone_id}",
                parameters=[
                    {
                        "image_size": IMAGE_SIZE,
                        "video_device": VIDEO_DEVICE,
                    },
                ],
                remappings=[
                    # Topics
                    ("camera_info", f"{zone_id}/camera_info"),
                    ("image_raw", f"{zone_id}/image_raw"),
                    ("image_rect", f"{zone_id}/image_rect"),
                    # Services
                    (
                        f"v4l2_camera_{zone_id}/set_camera_info",
                        f"{zone_id}/set_camera_info",
                    ),
                ],
            ),
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name=f"rectify_node_{zone_id}",
                parameters=[
                    {"image_transport": "compressed", "interpolation": 1},  # Linear
                ],
                remappings=[
                    # Topics
                    ("camera_info", f"{zone_id}/camera_info"),
                    ("image_rect", f"{zone_id}/image_rect"),
                    ("image", f"{zone_id}/image_raw"),
                ],
            ),
        ],
    )
    ld.add_action(video_container)


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    # Disabled to save resources
    # add_serial_port_scanner(ld, HOST_ID)
    add_system_monitor(ld, HOST_ID)

    # Smarthome hosts
    if HOST_ID in SMART_DISPLAY_ZONES:
        add_display_server(ld, ZONE_ID)

    if HOST_ID == "bar":
        add_v4l2_camera(ld, ZONE_ID)
    elif HOST_ID == "door":
        add_ros2_camera(ld, "doorbell")
        add_ros2_camera(ld, "entryway")
    elif HOST_ID == HOME_ASSISTANT_ID:
        add_home_assistant(ld)
    elif HOST_ID == "kitchen":
        add_v4l2_camera(ld, ZONE_ID)
    elif HOST_ID == "nas":
        add_kinect_v2(ld, KINECT_V2_ZONE_ID)

    # LEGO hosts
    if HOST_ID == "falcon":
        add_ros2_camera(ld, ZONE_ID)
    elif HOST_ID == "station":
        add_ros2_camera(ld, ZONE_ID)

    #
    # TODO: Microcontroller drivers
    #

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
    if HOST_ID == "station":
        mcu_node = "conductor"
        conductor_bridge_node: Node = get_telemetrix_bridge(HOST_ID, mcu_node)
        ld.add_action(conductor_bridge_node)
    elif HOST_ID == "jetson":
        mcu_node = "engine"
        engine_bridge_node: Node = get_telemetrix_bridge(HOST_ID, mcu_node)
        ld.add_action(engine_bridge_node)
    elif HOST_ID == "falcon":
        mcu_node = "engineer"
        engineer_bridge_node: Node = get_firmata_bridge(HOST_ID, mcu_node)
        ld.add_action(engineer_bridge_node)

    return ld
