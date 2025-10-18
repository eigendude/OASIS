################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import time
from datetime import datetime
from typing import List
from typing import Tuple

import rclpy.node
import rclpy.service
import rclpy.time
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Vector3 as Vector3Msg
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Header as HeaderMsg

from oasis_drivers.ros.ros_translator import RosTranslator
from oasis_drivers.telemetrix.telemetrix_bridge import TelemetrixBridge
from oasis_drivers.telemetrix.telemetrix_callback import TelemetrixCallback
from oasis_drivers.telemetrix.telemetrix_types import AnalogMode
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode
from oasis_msgs.msg import AirQuality as AirQualityMsg
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import AVRConstants as AVRConstantsMsg
from oasis_msgs.msg import CPUFanSpeed as CPUFanSpeedMsg
from oasis_msgs.msg import DigitalReading as DigitalReadingMsg
from oasis_msgs.msg import I2CDevice as I2CDeviceMsg
from oasis_msgs.msg import I2CDeviceType as I2CDeviceTypeMsg
from oasis_msgs.msg import I2CImu as I2CImuMsg
from oasis_msgs.msg import MCUMemory as MCUMemoryMsg
from oasis_msgs.msg import MCUString as MCUStringMsg
from oasis_msgs.srv import AnalogRead as AnalogReadSvc
from oasis_msgs.srv import DigitalRead as DigitalReadSvc
from oasis_msgs.srv import DigitalWrite as DigitalWriteSvc
from oasis_msgs.srv import I2CBegin as I2CBeginSvc
from oasis_msgs.srv import I2CEnd as I2CEndSvc
from oasis_msgs.srv import PWMWrite as PWMWriteSvc
from oasis_msgs.srv import ReportMCUMemory as ReportMCUMemorySvc
from oasis_msgs.srv import ServoWrite as ServoWriteSvc
from oasis_msgs.srv import SetAnalogMode as SetAnalogModeSvc
from oasis_msgs.srv import SetDigitalMode as SetDigitalModeSvc
from oasis_msgs.srv import SetSamplingInterval as SetSamplingIntervalSvc


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "telemetrix_bridge"

# ROS parameters
PARAM_COM_PORT = "com_port"

# ROS topics
AIR_QUALITY_TOPIC = "air_quality"
ANALOG_READING_TOPIC = "analog_reading"
CPU_FAN_SPEED_TOPIC = "cpu_fan_speed"
DIGITAL_READING_TOPIC = "digital_reading"
IMU_TOPIC = "i2c_imu"
MCU_MEMORY_TOPIC = "mcu_memory"
MCU_STRING_TOPIC = "mcu_string"

# ROS services
ANALOG_READ_SERVICE = "analog_read"
CPU_FAN_WRITE_SERVICE = "cpu_fan_write"
DIGITAL_READ_SERVICE = "digital_read"
DIGITAL_WRITE_SERVICE = "digital_write"
I2C_BEGIN_SERVICE = "i2c_begin"
I2C_END_SERVICE = "i2c_end"
PWM_WRITE_SERVICE = "pwm_write"
REPORT_MCU_MEMORY_SERVICE = "report_mcu_memory"
SERVO_WRITE_SERVICE = "servo_write"
SET_ANALOG_MODE_SERVICE = "set_analog_mode"
SET_CPU_FAN_SAMPLING_INTERVAL_SERVICE = "set_cpu_fan_sampling_interval"
SET_DIGITAL_MODE_SERVICE = "set_digital_mode"
SET_SAMPLING_INTERVAL_SERVICE = "set_sampling_interval"


################################################################################
# Telemetrix parameters
################################################################################


DEFAULT_COM_PORT = "/dev/ttyACM0"


################################################################################
# ROS node
################################################################################


class TelemetrixBridgeNode(rclpy.node.Node, TelemetrixCallback):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        # Initialize rclpy.node.NODE
        super().__init__(NODE_NAME)
        self.declare_parameter(PARAM_COM_PORT, DEFAULT_COM_PORT)

        # Initialize members
        com_port: str = str(self.get_parameter(PARAM_COM_PORT).value)
        self._bridge = TelemetrixBridge(self, com_port)
        self._initialized: bool = True

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._air_quality_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=AirQualityMsg,
            topic=AIR_QUALITY_TOPIC,
            qos_profile=qos_profile,
        )
        self._analog_reading_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=AnalogReadingMsg,
            topic=ANALOG_READING_TOPIC,
            qos_profile=qos_profile,
        )
        self._cpu_fan_speed_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=CPUFanSpeedMsg,
            topic=CPU_FAN_SPEED_TOPIC,
            qos_profile=qos_profile,
        )
        self._digital_reading_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=DigitalReadingMsg,
            topic=DIGITAL_READING_TOPIC,
            qos_profile=qos_profile,
        )
        self._imu_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=ImuMsg,
            topic=IMU_TOPIC,
            qos_profile=qos_profile,
        )
        self._mcu_memory_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=MCUMemoryMsg,
            topic=MCU_MEMORY_TOPIC,
            qos_profile=qos_profile,
        )
        self._mcu_string_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=MCUStringMsg,
            topic=MCU_STRING_TOPIC,
            qos_profile=qos_profile,
        )

        # Once the publishers are set up, start the bridge
        self._bridge.initialize()

        # Once the bridge is started, advertise the services
        self._analog_read_service: rclpy.service.Service = self.create_service(
            srv_type=AnalogReadSvc,
            srv_name=ANALOG_READ_SERVICE,
            callback=self._handle_analog_read,
        )
        self._cpu_fan_write_service: rclpy.service.Service = self.create_service(
            srv_type=PWMWriteSvc,
            srv_name=CPU_FAN_WRITE_SERVICE,
            callback=self._handle_cpu_fan_write,
        )
        self._digital_read_service: rclpy.service.Service = self.create_service(
            srv_type=DigitalReadSvc,
            srv_name=DIGITAL_READ_SERVICE,
            callback=self._handle_digital_read,
        )
        self._digital_write_service: rclpy.service.Service = self.create_service(
            srv_type=DigitalWriteSvc,
            srv_name=DIGITAL_WRITE_SERVICE,
            callback=self._handle_digital_write,
        )
        self._i2c_begin_service: rclpy.service.Service = self.create_service(
            srv_type=I2CBeginSvc,
            srv_name=I2C_BEGIN_SERVICE,
            callback=self._handle_i2c_begin,
        )
        self._i2c_end_service: rclpy.service.Service = self.create_service(
            srv_type=I2CEndSvc,
            srv_name=I2C_END_SERVICE,
            callback=self._handle_i2c_end,
        )
        self._pwm_write_service: rclpy.service.Service = self.create_service(
            srv_type=PWMWriteSvc,
            srv_name=PWM_WRITE_SERVICE,
            callback=self._handle_pwm_write,
        )
        self._report_mcu_memory_service: rclpy.service.Service = self.create_service(
            srv_type=ReportMCUMemorySvc,
            srv_name=REPORT_MCU_MEMORY_SERVICE,
            callback=self._handle_report_mcu_memory,
        )
        self._servo_write_service: rclpy.service.Service = self.create_service(
            srv_type=ServoWriteSvc,
            srv_name=SERVO_WRITE_SERVICE,
            callback=self._handle_servo_write,
        )
        self._set_analog_mode_service: rclpy.service.Service = self.create_service(
            srv_type=SetAnalogModeSvc,
            srv_name=SET_ANALOG_MODE_SERVICE,
            callback=self._handle_set_analog_mode,
        )
        self._set_cpu_fan_sampling_interval_service: rclpy.service.Service = (
            self.create_service(
                srv_type=SetSamplingIntervalSvc,
                srv_name=SET_CPU_FAN_SAMPLING_INTERVAL_SERVICE,
                callback=self._handle_set_cpu_fan_sampling_interval,
            )
        )
        self._set_digital_mode_service: rclpy.service.Service = self.create_service(
            srv_type=SetDigitalModeSvc,
            srv_name=SET_DIGITAL_MODE_SERVICE,
            callback=self._handle_set_digital_mode,
        )
        self._set_sampling_interval_service: rclpy.service.Service = (
            self.create_service(
                srv_type=SetSamplingIntervalSvc,
                srv_name=SET_SAMPLING_INTERVAL_SERVICE,
                callback=self._handle_set_sampling_interval,
            )
        )

        self.get_logger().info("Telemetrix bridge initialized")

    def stop(self) -> None:
        """Stop the bridge and cleanup ROS resources"""
        self._initialized = False
        self._bridge.deinitialize()

        # Wait for outstanding tasks to complete
        time.sleep(1)

        self.get_logger().info("Telemetrix bridge deinitialized")

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        self.destroy_node()

    def on_air_quality(
        self,
        timestamp: datetime,
        i2c_port: int,
        i2c_address: int,
        co2_ppb: int,
        tvoc_ppb: int,
    ) -> None:
        """Implement TelemetrixCallback"""
        if not self._initialized:
            return

        msg: AirQualityMsg = AirQualityMsg()

        # Timestamp in ROS header
        header: HeaderMsg = HeaderMsg()
        header.stamp = RosTranslator.convert_timestamp(timestamp)
        header.frame_id = ""  # TODO

        msg.header = header
        msg.co2_ppb = float(co2_ppb)
        msg.tvoc_ppb = float(tvoc_ppb)

        i2c_device: I2CDeviceMsg = I2CDeviceMsg()
        i2c_device.i2c_device_type = I2CDeviceTypeMsg.CCS811
        i2c_device.i2c_port = i2c_port
        i2c_device.i2c_address = i2c_address
        msg.i2c_device.append(i2c_device)

        self._air_quality_pub.publish(msg)

    def on_analog_reading(
        self,
        timestamp: datetime,
        analog_pin: int,
        analog_value: float,
        reference_voltage: float,
    ) -> None:
        """Implement TelemetrixCallback"""
        if not self._initialized:
            return

        msg: AnalogReadingMsg = AnalogReadingMsg()

        # Timestamp in ROS header
        header = HeaderMsg()
        header.stamp = RosTranslator.convert_timestamp(timestamp)
        header.frame_id = ""  # TODO

        msg.header = header
        msg.analog_pin = analog_pin
        msg.reference_voltage = reference_voltage
        msg.analog_value = analog_value

        self._analog_reading_pub.publish(msg)

    def on_cpu_fan_rpm(self, timestamp: datetime, digital_pin: int, rpm: int) -> None:
        """Implement TelemetrixCallback"""
        if not self._initialized:
            return

        msg: CPUFanSpeedMsg = CPUFanSpeedMsg()

        # Timestamp in ROS header
        header = HeaderMsg()
        header.stamp = RosTranslator.convert_timestamp(timestamp)
        header.frame_id = ""  # TODO

        msg.header = header
        msg.digital_pin = digital_pin
        msg.fan_speed_rpm = float(rpm)

        self._cpu_fan_speed_pub.publish(msg)

    def on_digital_reading(
        self, timestamp: datetime, digital_pin: int, digital_value: bool
    ) -> None:
        """Implement TelemetrixCallback"""
        if not self._initialized:
            return

        msg: DigitalReadingMsg = DigitalReadingMsg()

        # Timestamp in ROS header
        header = HeaderMsg()
        header.stamp = RosTranslator.convert_timestamp(timestamp)
        header.frame_id = ""  # TODO

        msg.header = header
        msg.digital_pin = digital_pin
        msg.digital_value = (
            AVRConstantsMsg.HIGH if digital_value else AVRConstantsMsg.LOW
        )

        self._digital_reading_pub.publish(msg)

    def on_imu_6_axis(
        self,
        timestamp: datetime,
        i2c_port: int,
        i2c_address: int,
        ax: int,
        ay: int,
        az: int,
        gx: int,
        gy: int,
        gz: int,
    ) -> None:
        """Implement TelemetrixCallback"""
        if not self._initialized:
            return

        imu_msg: ImuMsg = ImuMsg()
        # Timestamp in ROS header
        imu_msg.header.stamp = RosTranslator.convert_timestamp(timestamp)
        imu_msg.header.frame_id = ""  # TODO

        imu_msg.angular_velocity = Vector3Msg()
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.x = gy
        imu_msg.angular_velocity.x = gz

        imu_msg.linear_acceleration = Vector3Msg()
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        # TODO: Covariance matrices

        i2c_imu_msg: I2CImuMsg = I2CImuMsg()
        i2c_imu_msg.i2c_device.i2c_device_type = I2CDeviceTypeMsg.MPU6050
        i2c_imu_msg.i2c_device.i2c_port = i2c_port
        i2c_imu_msg.i2c_device.i2c_address = i2c_address
        i2c_imu_msg.imu = imu_msg

        self._imu_publish_pub.publish(i2c_imu_msg)

    def on_memory_data(
        self,
        total_ram: int,
        static_data_size: int,
        heap_size: int,
        stack_size: int,
        free_ram: int,
        free_heap: int,
    ) -> None:
        """Implement TelemetrixCallback"""
        if not self._initialized:
            return

        msg: MCUMemoryMsg = MCUMemoryMsg()

        # Timestamp in ROS header
        header = HeaderMsg()
        header.stamp = self._get_timestamp()
        header.frame_id = ""  # TODO

        msg.header = header
        msg.total_ram = total_ram
        msg.static_data_size = static_data_size
        msg.heap_size = heap_size
        msg.stack_size = stack_size
        msg.free_ram = free_ram
        msg.free_heap = free_heap

        self._mcu_memory_pub.publish(msg)

    def on_string_data(self, data: str) -> None:
        """Implement TelemetrixCallback"""
        if not self._initialized:
            return

        msg: MCUStringMsg = MCUStringMsg()

        # Timestamp in ROS header
        header = HeaderMsg()
        header.stamp = self._get_timestamp()
        header.frame_id = ""  # TODO

        msg.header = header
        msg.message = data

        self._mcu_string_pub.publish(msg)

        # Debug logging
        self.get_logger().info(data)

    def _handle_analog_read(
        self, request: AnalogReadSvc.Request, response: AnalogReadSvc.Response
    ) -> AnalogReadSvc.Response:
        """Handle ROS 2 analog pin reads"""
        # Translate parameters
        analog_pin: int = request.analog_pin

        # Debug logging
        self.get_logger().info(f"Reading value for analog pin {analog_pin}")

        # Perform service
        result: Tuple[float, float, datetime] = self._bridge.analog_read(analog_pin)

        # Translate result
        response.stamp = RosTranslator.convert_timestamp(result[2])
        response.reference_voltage = float(result[1])
        response.analog_value = float(result[0])

        return response

    def _handle_cpu_fan_write(
        self, request: PWMWriteSvc.Request, response: PWMWriteSvc.Response
    ) -> PWMWriteSvc.Response:
        """Handle ROS 2 CPU fan PWM writes"""
        # Translate parameters
        digital_pin: int = request.digital_pin
        duty_cycle: float = request.duty_cycle

        # Debug logging
        self.get_logger().info(
            f"Setting CPU fan on pin {digital_pin} to duty cycle {duty_cycle}"
        )

        # Perform service
        self._bridge.cpu_fan_write(digital_pin, duty_cycle)

        return response

    def _handle_digital_read(
        self, request: DigitalReadSvc.Request, response: DigitalReadSvc.Response
    ) -> DigitalReadSvc.Response:
        """Handle ROS 2 digital pin reads"""
        # Translate parameters
        digital_pin: int = request.digital_pin

        # Debug logging
        self.get_logger().info(f"Reading value for digital pin {digital_pin}")

        # Perform service
        result: Tuple[bool, datetime] = self._bridge.digital_read(digital_pin)

        # Translate result
        response.stamp = RosTranslator.convert_timestamp(result[1])
        response.value = AVRConstantsMsg.HIGH if result[0] else AVRConstantsMsg.LOW

        return response

    def _handle_digital_write(
        self, request: DigitalWriteSvc.Request, response: DigitalWriteSvc.Response
    ) -> DigitalWriteSvc.Response:
        """Handle ROS 2 digital pin writes"""
        # Translate parameters
        digital_pin: int = request.digital_pin
        digital_value: bool = request.digital_value == AVRConstantsMsg.HIGH

        # Debug logging
        self.get_logger().info(
            f"Setting digital pin {digital_pin} value to {'HIGH' if digital_value else 'LOW'}"
        )

        # Perform service
        self._bridge.digital_write(digital_pin, digital_value)

        return response

    def _handle_i2c_begin(
        self, request: I2CBeginSvc.Request, response: I2CBeginSvc.Response
    ) -> I2CBeginSvc.Response:
        """Begin I2C communication on a port and for devices"""
        # Translate parameters
        i2c_port: int = request.i2c_port
        i2c_devices: List[I2CDeviceMsg] = request.i2c_devices

        # Debug logging
        self.get_logger().info(f"Beginning I2C on port {i2c_port}")

        # Perform service for port
        self._bridge.i2c_begin(i2c_port)

        # Perform services for devices
        for i2c_device in i2c_devices:
            i2c_device_type: int = i2c_device.i2c_device_type
            i2c_address: int = i2c_device.i2c_address

            self.get_logger().info(
                f"Beginning I2C device of type {i2c_device_type} with address {hex(i2c_address)}"
            )
            if i2c_device_type == I2CDeviceTypeMsg.CCS811:
                self._bridge.i2c_ccs811_begin(i2c_port, i2c_address)
            elif i2c_device_type == I2CDeviceTypeMsg.MPU6050:
                self._bridge.i2c_mpu6050_begin(i2c_port, i2c_address)

        return response

    def _handle_i2c_end(
        self, request: I2CEndSvc.Request, response: I2CEndSvc.Response
    ) -> I2CEndSvc.Response:
        """End I2C communication on a port and for devices"""
        # Translate parameters
        i2c_port: int = request.i2c_port
        i2c_devices: List[I2CDeviceMsg] = request.i2c_devices

        # Debug logging
        self.get_logger().info(f"Ending I2C on port {i2c_port}")

        # Perform services for devices
        for i2c_device in i2c_devices:
            i2c_device_type: int = i2c_device.i2c_device_type
            i2c_address: int = i2c_device.i2c_address

            self.get_logger().info(
                f"Ending I2C device of type {i2c_device_type} with address {hex(i2c_address)}"
            )
            if i2c_device_type == I2CDeviceTypeMsg.CCS811:
                self._bridge.i2c_ccs811_end(i2c_port, i2c_address)
            elif i2c_device_type == I2CDeviceTypeMsg.MPU6050:
                self._bridge.i2c_mpu6050_end(i2c_port, i2c_address)

        return response

    def _handle_pwm_write(
        self, request: PWMWriteSvc.Request, response: PWMWriteSvc.Response
    ) -> PWMWriteSvc.Response:
        """Handle ROS 2 PWM pin writes"""
        # Translate parameters
        digital_pin: int = request.digital_pin
        duty_cycle: float = request.duty_cycle

        # Debug logging
        self.get_logger().info(
            f"Setting PWM on pin {digital_pin} to duty cycle {duty_cycle}"
        )

        # Perform service
        self._bridge.pwm_write(digital_pin, duty_cycle)

        return response

    def _handle_report_mcu_memory(
        self, request: ReportMCUMemorySvc.Request, response: ReportMCUMemorySvc.Response
    ) -> ReportMCUMemorySvc.Response:
        """Handle request to enable/disable MCU memory reporting"""
        # Translate parameters
        reporting_period_ms: int = request.reporting_period_ms

        # Debug logging
        if reporting_period_ms != 0:
            self.get_logger().info(f"Reporting MCU memory at {reporting_period_ms}ms")
        else:
            self.get_logger().info("Disabling MCU memory reporting")

        # Perform service
        self._bridge.set_memory_reporting_interval(reporting_period_ms)

        return response

    def _handle_servo_write(
        self, request: ServoWriteSvc.Request, response: ServoWriteSvc.Response
    ) -> ServoWriteSvc.Response:
        """Handle ROS 2 servo pin writes"""
        # Translate parameters
        digital_pin: int = request.digital_pin
        position: float = request.position

        # Debug logging
        self.get_logger().info(
            f"Setting servo on pin {digital_pin} to position {position}"
        )

        # Perform service
        self._bridge.servo_write(digital_pin, position)

        return response

    def _handle_set_analog_mode(
        self, request: SetAnalogModeSvc.Request, response: SetAnalogModeSvc.Response
    ) -> SetAnalogModeSvc.Response:
        """Handle ROS 2 analog pin mode changes"""
        # Translate parameters
        analog_pin: int = request.analog_pin
        analog_mode: AnalogMode

        try:
            analog_mode = RosTranslator.analog_mode_to_telemetrix(request.analog_mode)
        except KeyError:
            self.get_logger().error(
                f"Invalid analog mode ({request.analog_mode}), disabling pin"
            )
            analog_mode = AnalogMode.DISABLED

        # Debug logging
        self.get_logger().info(f"Setting analog pin {analog_pin} to mode {analog_mode}")

        # Perform service
        self._bridge.set_analog_mode(analog_pin, analog_mode)

        return response

    def _handle_set_cpu_fan_sampling_interval(
        self,
        request: SetSamplingIntervalSvc.Request,
        response: SetSamplingIntervalSvc.Response,
    ) -> SetSamplingIntervalSvc.Response:
        """Handle ROS 2 CPU fan sampling interval changes"""
        # Translate parameters
        sampling_interval_ms: int = request.sampling_interval_ms

        # Debug logging
        self.get_logger().info(
            f"Setting CPU fan sampling interval to {sampling_interval_ms} ms"
        )

        # Perform service
        self._bridge.set_cpu_fan_sampling_interval(sampling_interval_ms)

        return response

    def _handle_set_digital_mode(
        self, request: SetDigitalModeSvc.Request, response: SetDigitalModeSvc.Response
    ) -> SetDigitalModeSvc.Response:
        """Handle ROS 2 digital pin mode changes"""
        # Translate parameters
        digital_pin: int = request.digital_pin
        digital_mode: DigitalMode

        try:
            digital_mode = RosTranslator.digital_mode_to_telemetrix(
                request.digital_mode
            )
        except KeyError:
            self.get_logger().error(
                f"Invalid digital mode ({request.digital_mode}), disabling pin"
            )
            digital_mode = DigitalMode.DISABLED

        # Debug logging
        self.get_logger().info(
            f"Setting digital pin {digital_pin} to mode {digital_mode}"
        )

        # Perform service
        self._bridge.set_digital_mode(digital_pin, digital_mode)

        return response

    def _handle_set_sampling_interval(
        self,
        request: SetSamplingIntervalSvc.Request,
        response: SetSamplingIntervalSvc.Response,
    ) -> SetSamplingIntervalSvc.Response:
        """Handle ROS 2 sampling interval changes"""
        # Translate parameters
        sampling_interval_ms: int = request.sampling_interval_ms

        # Debug logging
        self.get_logger().info(
            f"Setting sampling interval to {sampling_interval_ms} ms"
        )

        # Perform service
        self._bridge.set_sampling_interval(sampling_interval_ms)

        return response

    def _get_timestamp(self) -> TimeMsg:
        return self.get_clock().now().to_msg()
