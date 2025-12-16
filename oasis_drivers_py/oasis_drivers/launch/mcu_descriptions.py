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

PYTHON_PACKAGE_NAME: str = "oasis_drivers_py"


################################################################################
# Microcontroller nodes
################################################################################


class MCUDescriptions:
    @staticmethod
    def add_firmata_bridge(
        ld: LaunchDescription, hostname: str, mcu_name: str, avr_com_port: str
    ) -> None:
        firmata_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="firmata_bridge",
            name=f"{mcu_name}_bridge_{hostname}",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "com_port": avr_com_port,
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
        ld.add_action(firmata_node)

    @staticmethod
    def add_telemetrix_bridge(
        ld: LaunchDescription, hostname: str, mcu_name: str, avr_com_port: str
    ) -> None:
        telemetrix_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="telemetrix_bridge",
            name=f"{mcu_name}_bridge_{hostname}",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "com_port": avr_com_port,
                },
            ],
            remappings=[
                ("air_quality", f"{mcu_name}/air_quality"),
                ("analog_read", f"{mcu_name}/analog_read"),
                ("analog_reading", f"{mcu_name}/analog_reading"),
                ("cpu_fan_speed", f"{mcu_name}/cpu_fan_speed"),
                ("cpu_fan_write", f"{mcu_name}/cpu_fan_write"),
                ("cpu_fan_write_cmd", f"{mcu_name}/cpu_fan_write_cmd"),
                ("digital_read", f"{mcu_name}/digital_read"),
                ("digital_reading", f"{mcu_name}/digital_reading"),
                ("digital_write", f"{mcu_name}/digital_write"),
                ("digital_write_cmd", f"{mcu_name}/digital_write_cmd"),
                ("i2c_begin", f"{mcu_name}/i2c_begin"),
                ("i2c_end", f"{mcu_name}/i2c_end"),
                ("i2c_imu", f"{mcu_name}/i2c_imu"),
                ("mcu_memory", f"{mcu_name}/mcu_memory"),
                ("mcu_string", f"{mcu_name}/mcu_string"),
                ("pwm_write", f"{mcu_name}/pwm_write"),
                ("pwm_write_cmd", f"{mcu_name}/pwm_write_cmd"),
                ("report_mcu_memory", f"{mcu_name}/report_mcu_memory"),
                ("servo_write", f"{mcu_name}/servo_write"),
                ("servo_write_cmd", f"{mcu_name}/servo_write_cmd"),
                ("set_analog_mode", f"{mcu_name}/set_analog_mode"),
                (
                    "set_cpu_fan_sampling_interval",
                    f"{mcu_name}/set_cpu_fan_sampling_interval",
                ),
                ("set_digital_mode", f"{mcu_name}/set_digital_mode"),
                ("set_sampling_interval", f"{mcu_name}/set_sampling_interval"),
            ],
        )
        ld.add_action(telemetrix_node)
