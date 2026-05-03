################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Install lightweight ROS and launch stubs for non-ROS test environments."""

from __future__ import annotations

import importlib
import sys
from types import ModuleType
from typing import Any


def install_test_stubs() -> None:
    """Install test-only stand-ins for ROS packages that may be absent."""

    _install_builtin_interfaces_stub()
    _install_geometry_msgs_stub()
    _install_sensor_msgs_stub()
    _install_std_msgs_stub()
    _install_nav_msgs_stub()
    _install_tf2_ros_stub()
    _install_oasis_msgs_stub()
    _install_launch_stub()
    _install_launch_ros_stub()
    _install_rclpy_stub()


def _module_exists(module_name: str) -> bool:
    try:
        importlib.import_module(module_name)
    except ModuleNotFoundError:
        return False

    return True


def _register_module(module_name: str, module: ModuleType) -> None:
    sys.modules[module_name] = module


def _set_module_attr(module: ModuleType, name: str, value: Any) -> None:
    setattr(module, name, value)


def _make_package(module_name: str) -> ModuleType:
    module: ModuleType = ModuleType(module_name)
    module.__path__ = []
    return module


class _Time:
    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.sec: int = sec
        self.nanosec: int = nanosec


class _Header:
    def __init__(self) -> None:
        self.stamp: _Time = _Time()
        self.frame_id: str = ""


class _Bool:
    def __init__(self) -> None:
        self.data: bool = False


class _String:
    def __init__(self) -> None:
        self.data: str = ""


class _AVRConstants:
    ANALOG_DISABLED: int = 0
    ANALOG_INPUT: int = 1

    DIGITAL_DISABLED: int = 0
    DIGITAL_INPUT: int = 1
    DIGITAL_INPUT_PULLUP: int = 2
    DIGITAL_OUTPUT: int = 3
    DIGITAL_PWM: int = 4
    DIGITAL_SERVO: int = 5
    DIGITAL_CPU_FAN_PWM: int = 6
    DIGITAL_CPU_FAN_TACHOMETER: int = 7

    LOW: int = 0
    HIGH: int = 1


class _AnalogReading:
    def __init__(self) -> None:
        self.analog_pin: int = 0
        self.analog_value: float = 0.0


class _AnalogReadings:
    def __init__(self) -> None:
        self.header: _Header = _Header()
        self.readings: list[_AnalogReading] = []


class _DigitalReading:
    def __init__(self) -> None:
        self.digital_pin: int = 0
        self.digital_value: int = 0


class _DigitalWriteCommand:
    def __init__(self) -> None:
        self.digital_pin: int = 0
        self.digital_value: int = 0


class _PWMWriteCommand:
    def __init__(self) -> None:
        self.digital_pin: int = 0
        self.pwm_value: float = 0.0


class _SerialDevice:
    def __init__(self) -> None:
        self.device: str = ""
        self.name: str = ""
        self.description: str = ""
        self.hardware_id: str = ""
        self.usb_device: list[_UsbDevice] = []


class _SerialDeviceScan:
    def __init__(self) -> None:
        self.header: _Header = _Header()
        self.serial_devices: list[_SerialDevice] = []


class _UsbDevice:
    def __init__(self) -> None:
        self.vendor_id: int = 0
        self.product_id: int = 0
        self.serial_number: str = ""
        self.location: str = ""
        self.manufacturer: str = ""
        self.product: str = ""


class _Vector3:
    def __init__(self) -> None:
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0


class _Quaternion:
    def __init__(self) -> None:
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0
        self.w: float = 1.0


class _Point:
    def __init__(self) -> None:
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0


class _Transform:
    def __init__(self) -> None:
        self.translation: _Vector3 = _Vector3()
        self.rotation: _Quaternion = _Quaternion()


class _TransformStamped:
    def __init__(self) -> None:
        self.header: _Header = _Header()
        self.child_frame_id: str = ""
        self.transform: _Transform = _Transform()


class _Accel:
    def __init__(self) -> None:
        self.linear: _Vector3 = _Vector3()


class _AccelWithCovariance:
    def __init__(self) -> None:
        self.accel: _Accel = _Accel()
        self.covariance: list[float] = [0.0] * 36


class _AccelWithCovarianceStamped:
    def __init__(self) -> None:
        self.header: _Header = _Header()
        self.accel: _AccelWithCovariance = _AccelWithCovariance()


class _Imu:
    def __init__(self) -> None:
        self.header: _Header = _Header()
        self.orientation: _Quaternion = _Quaternion()
        self.orientation_covariance: list[float] = [0.0] * 9
        self.angular_velocity: _Vector3 = _Vector3()
        self.angular_velocity_covariance: list[float] = [0.0] * 9
        self.linear_acceleration: _Vector3 = _Vector3()
        self.linear_acceleration_covariance: list[float] = [0.0] * 9


class _BatteryState:
    POWER_SUPPLY_STATUS_UNKNOWN: int = 0
    POWER_SUPPLY_STATUS_CHARGING: int = 1
    POWER_SUPPLY_STATUS_DISCHARGING: int = 2
    POWER_SUPPLY_STATUS_NOT_CHARGING: int = 3
    POWER_SUPPLY_STATUS_FULL: int = 4

    POWER_SUPPLY_HEALTH_UNKNOWN: int = 0
    POWER_SUPPLY_HEALTH_GOOD: int = 1
    POWER_SUPPLY_HEALTH_OVERHEAT: int = 2
    POWER_SUPPLY_HEALTH_DEAD: int = 3
    POWER_SUPPLY_HEALTH_OVERVOLTAGE: int = 4
    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: int = 5
    POWER_SUPPLY_HEALTH_COLD: int = 6
    POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: int = 7
    POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: int = 8

    POWER_SUPPLY_TECHNOLOGY_UNKNOWN: int = 0
    POWER_SUPPLY_TECHNOLOGY_NIMH: int = 1
    POWER_SUPPLY_TECHNOLOGY_LION: int = 2
    POWER_SUPPLY_TECHNOLOGY_LIPO: int = 3
    POWER_SUPPLY_TECHNOLOGY_LIFE: int = 4
    POWER_SUPPLY_TECHNOLOGY_NICD: int = 5
    POWER_SUPPLY_TECHNOLOGY_LIMN: int = 6

    def __init__(self) -> None:
        self.header: _Header = _Header()
        self.voltage: float = 0.0
        self.temperature: float = 0.0
        self.current: float = 0.0
        self.charge: float = 0.0
        self.capacity: float = 0.0
        self.design_capacity: float = 0.0
        self.percentage: float = 0.0
        self.power_supply_status: int = self.POWER_SUPPLY_STATUS_UNKNOWN
        self.power_supply_health: int = self.POWER_SUPPLY_HEALTH_UNKNOWN
        self.power_supply_technology: int = self.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self.present: bool = False
        self.cell_voltage: list[float] = []
        self.cell_temperature: list[float] = []
        self.location: str = ""
        self.serial_number: str = ""


class _Pose:
    def __init__(self) -> None:
        self.position: _Point = _Point()
        self.orientation: _Quaternion = _Quaternion()


class _PoseWithCovariance:
    def __init__(self) -> None:
        self.pose: _Pose = _Pose()
        self.covariance: list[float] = [0.0] * 36


class _Twist:
    def __init__(self) -> None:
        self.linear: _Vector3 = _Vector3()
        self.angular: _Vector3 = _Vector3()


class _TwistWithCovariance:
    def __init__(self) -> None:
        self.twist: _Twist = _Twist()
        self.covariance: list[float] = [0.0] * 36


class _TwistWithCovarianceStamped:
    def __init__(self) -> None:
        self.header: _Header = _Header()
        self.twist: _TwistWithCovariance = _TwistWithCovariance()


class _Odometry:
    def __init__(self) -> None:
        self.header: _Header = _Header()
        self.child_frame_id: str = ""
        self.pose: _PoseWithCovariance = _PoseWithCovariance()
        self.twist: _TwistWithCovariance = _TwistWithCovariance()


class _AhrsStatus:
    STATUS_OK: int = 0
    STATUS_WAITING_FOR_IMU: int = 1
    STATUS_WAITING_FOR_GRAVITY: int = 2
    STATUS_BAD_IMU_FRAME: int = 3
    STATUS_BAD_GRAVITY_FRAME: int = 4
    STATUS_MOUNTING_UNAVAILABLE: int = 5

    def __init__(self) -> None:
        self.header: _Header = _Header()
        self.status: int = self.STATUS_WAITING_FOR_IMU
        self.accepted_imu_count: int = 0
        self.accepted_gravity_count: int = 0
        self.rejected_imu_count: int = 0
        self.rejected_gravity_count: int = 0
        self.dropped_stale_imu_count: int = 0
        self.dropped_stale_gravity_count: int = 0
        self.gravity_rejection_count: int = 0
        self.gravity_residual_norm: float = 0.0
        self.gravity_mahalanobis_distance: float = 0.0
        self.has_gravity: bool = False
        self.has_mounting: bool = False
        self.gravity_gated_in: bool = False
        self.gravity_rejected: bool = False
        self.transform_lookup_failure_count: int = 0
        self.invalid_mounting_transform_count: int = 0
        self.last_mounting_lookup_error: str = ""
        self.last_gravity_rejection_reason: str = ""
        self.status_text: str = ""


class _UPSStatus:
    def __init__(
        self,
        manufacturer: str = "",
        model: str = "",
        serial_number: str = "",
        input_voltage: float = 0.0,
        output_voltage: float = 0.0,
        battery_voltage: float = 0.0,
        battery_current: float = 0.0,
        battery_temperature: float = 0.0,
        battery_charge: int = 0,
        battery_runtime: int = 0,
        load: float = 0.0,
        load_total: float = 0.0,
        status: str = "",
        battery_type: str = "",
    ) -> None:
        self.manufacturer: str = manufacturer
        self.model: str = model
        self.serial_number: str = serial_number
        self.input_voltage: float = input_voltage
        self.output_voltage: float = output_voltage
        self.battery_voltage: float = battery_voltage
        self.battery_current: float = battery_current
        self.battery_temperature: float = battery_temperature
        self.battery_charge: int = battery_charge
        self.battery_runtime: int = battery_runtime
        self.load: float = load
        self.load_total: float = load_total
        self.status: str = status
        self.battery_type: str = battery_type


class _UPSCommand:
    class Request:
        def __init__(self) -> None:
            self.command: str = ""
            self.delay: float = 0.0

    class Response:
        pass


class _SetAnalogMode:
    class Request:
        def __init__(self) -> None:
            self.analog_pin: int = 0
            self.analog_mode: int = 0

    class Response:
        pass


class _SetDigitalMode:
    class Request:
        def __init__(self) -> None:
            self.digital_pin: int = 0
            self.digital_mode: int = 0

    class Response:
        pass


class _Future:
    def result(self) -> object:
        return object()

    def exception(self) -> None:
        return None


class _Client:
    def __init__(self, *_: Any, **__: Any) -> None:
        pass

    def call_async(self, request: object) -> _Future:
        del request
        return _Future()


class _Publisher:
    def __init__(self, *_: Any, **__: Any) -> None:
        self.messages: list[Any] = []

    def publish(self, message: Any) -> None:
        self.messages.append(message)


class _Subscription:
    def __init__(self, *_: Any, **__: Any) -> None:
        pass


class _Service:
    def __init__(
        self,
        srv_type: type[Any] | None = None,
        srv_name: str = "",
        callback: Any = None,
    ) -> None:
        self.srv_type: type[Any] | None = srv_type
        self.srv_name: str = srv_name
        self.callback: Any = callback


class _Timer:
    def __init__(self, period_sec: float, callback: Any) -> None:
        self.period_sec: float = period_sec
        self.callback: Any = callback


class _Parameter:
    def __init__(self, value: Any) -> None:
        self.value: Any = value


class _Logger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def debug(self, message: str) -> None:
        self.messages.append(("debug", message))

    def info(self, message: str) -> None:
        self.messages.append(("info", message))

    def warning(self, message: str) -> None:
        self.messages.append(("warning", message))

    def error(self, message: str) -> None:
        self.messages.append(("error", message))


class _ClockNow:
    def to_msg(self) -> _Time:
        return _Time()


class _Clock:
    def now(self) -> _ClockNow:
        return _ClockNow()


class _Node:
    def __init__(self, name: str) -> None:
        self._name: str = name
        self._parameters: dict[str, _Parameter] = {}
        self._logger: _Logger = _Logger()

    def declare_parameter(self, name: str, default_value: Any) -> _Parameter:
        parameter: _Parameter = _Parameter(default_value)
        self._parameters[name] = parameter
        return parameter

    def get_parameter(self, name: str) -> _Parameter:
        return self._parameters[name]

    def create_publisher(
        self,
        msg_type: type[Any],
        topic: str,
        qos_profile: Any,
    ) -> _Publisher:
        del msg_type, topic, qos_profile
        return _Publisher()

    def create_subscription(
        self,
        msg_type: type[Any],
        topic: str,
        callback: Any,
        qos_profile: Any,
    ) -> _Subscription:
        del msg_type, topic, callback, qos_profile
        return _Subscription()

    def create_service(
        self,
        srv_type: type[Any],
        srv_name: str,
        callback: Any,
    ) -> _Service:
        return _Service(
            srv_type=srv_type,
            srv_name=srv_name,
            callback=callback,
        )

    def create_client(
        self,
        srv_type: type[Any],
        srv_name: str,
    ) -> _Client:
        del srv_type, srv_name
        return _Client()

    def create_timer(self, period_sec: float, callback: Any) -> _Timer:
        return _Timer(period_sec, callback)

    def destroy_node(self) -> None:
        return None

    def get_logger(self) -> _Logger:
        return self._logger

    def get_clock(self) -> _Clock:
        return _Clock()


class _QoSProfile:
    def __init__(self, *_: Any, **__: Any) -> None:
        pass


class _PresetProfile:
    def __init__(self) -> None:
        self.value: _QoSProfile = _QoSProfile()


class _QoSPresetProfiles:
    SENSOR_DATA: _PresetProfile = _PresetProfile()
    SYSTEM_DEFAULT: _PresetProfile = _PresetProfile()


class _QoSReliabilityPolicy:
    RELIABLE: int = 1


class _QoSHistoryPolicy:
    KEEP_LAST: int = 1


class _TimeHandle:
    pass


class _TransformException(Exception):
    pass


class _Buffer:
    def lookup_transform(self, target_frame: str, source_frame: str, time: Any) -> Any:
        del target_frame, source_frame, time
        raise _TransformException("transform unavailable")


class _TransformListener:
    def __init__(self, buffer: Any, node: Any) -> None:
        self.buffer: Any = buffer
        self.node: Any = node


class _TransformBroadcaster:
    def __init__(self, node: Any) -> None:
        self.node: Any = node
        self.transforms: list[Any] = []

    def sendTransform(self, transforms: Any) -> None:
        self.transforms.append(transforms)


class _LaunchDescription:
    def __init__(self) -> None:
        self.actions: list[Any] = []

    def add_action(self, action: Any) -> None:
        self.actions.append(action)


class _LaunchNode:
    def __init__(self, **kwargs: Any) -> None:
        self.kwargs: dict[str, Any] = kwargs


def _install_builtin_interfaces_stub() -> None:
    if _module_exists("builtin_interfaces.msg"):
        return

    builtin_interfaces_module: ModuleType = _make_package("builtin_interfaces")
    builtin_interfaces_msg_module: ModuleType = ModuleType("builtin_interfaces.msg")
    _set_module_attr(builtin_interfaces_msg_module, "Time", _Time)
    _set_module_attr(builtin_interfaces_module, "msg", builtin_interfaces_msg_module)
    _register_module("builtin_interfaces", builtin_interfaces_module)
    _register_module("builtin_interfaces.msg", builtin_interfaces_msg_module)


def _install_geometry_msgs_stub() -> None:
    if _module_exists("geometry_msgs.msg"):
        return

    geometry_msgs_module: ModuleType = _make_package("geometry_msgs")
    geometry_msgs_msg_module: ModuleType = ModuleType("geometry_msgs.msg")
    _set_module_attr(
        geometry_msgs_msg_module,
        "AccelWithCovarianceStamped",
        _AccelWithCovarianceStamped,
    )
    _set_module_attr(geometry_msgs_msg_module, "TransformStamped", _TransformStamped)
    _set_module_attr(
        geometry_msgs_msg_module,
        "TwistWithCovarianceStamped",
        _TwistWithCovarianceStamped,
    )
    _set_module_attr(geometry_msgs_module, "msg", geometry_msgs_msg_module)
    _register_module("geometry_msgs", geometry_msgs_module)
    _register_module("geometry_msgs.msg", geometry_msgs_msg_module)


def _install_sensor_msgs_stub() -> None:
    if _module_exists("sensor_msgs.msg"):
        return

    sensor_msgs_module: ModuleType = _make_package("sensor_msgs")
    sensor_msgs_msg_module: ModuleType = ModuleType("sensor_msgs.msg")
    _set_module_attr(sensor_msgs_msg_module, "Imu", _Imu)
    _set_module_attr(sensor_msgs_module, "msg", sensor_msgs_msg_module)
    _register_module("sensor_msgs", sensor_msgs_module)
    _register_module("sensor_msgs.msg", sensor_msgs_msg_module)


def _install_std_msgs_stub() -> None:
    if _module_exists("std_msgs") and _module_exists("std_msgs.msg"):
        std_msgs_module = importlib.import_module("std_msgs")
        std_msgs_msg_module = importlib.import_module("std_msgs.msg")
    else:
        std_msgs_module = _make_package("std_msgs")
        std_msgs_msg_module = ModuleType("std_msgs.msg")

    _set_module_attr(std_msgs_msg_module, "Bool", _Bool)
    _set_module_attr(std_msgs_msg_module, "Header", _Header)
    _set_module_attr(std_msgs_msg_module, "String", _String)
    _set_module_attr(std_msgs_module, "msg", std_msgs_msg_module)
    _register_module("std_msgs", std_msgs_module)
    _register_module("std_msgs.msg", std_msgs_msg_module)


def _install_nav_msgs_stub() -> None:
    if _module_exists("nav_msgs.msg"):
        return

    nav_msgs_module: ModuleType = _make_package("nav_msgs")
    nav_msgs_msg_module: ModuleType = ModuleType("nav_msgs.msg")
    _set_module_attr(nav_msgs_msg_module, "Odometry", _Odometry)
    _set_module_attr(nav_msgs_module, "msg", nav_msgs_msg_module)
    _register_module("nav_msgs", nav_msgs_module)
    _register_module("nav_msgs.msg", nav_msgs_msg_module)


def _install_tf2_ros_stub() -> None:
    if _module_exists("tf2_ros"):
        return

    tf2_ros_module: ModuleType = ModuleType("tf2_ros")
    _set_module_attr(tf2_ros_module, "Buffer", _Buffer)
    _set_module_attr(tf2_ros_module, "TransformBroadcaster", _TransformBroadcaster)
    _set_module_attr(tf2_ros_module, "TransformException", _TransformException)
    _set_module_attr(tf2_ros_module, "TransformListener", _TransformListener)
    _register_module("tf2_ros", tf2_ros_module)


def _install_oasis_msgs_stub() -> None:
    if (
        _module_exists("oasis_msgs")
        and _module_exists("oasis_msgs.msg")
        and _module_exists("oasis_msgs.srv")
    ):
        oasis_msgs_module = importlib.import_module("oasis_msgs")
        oasis_msgs_msg_module = importlib.import_module("oasis_msgs.msg")
        oasis_msgs_srv_module = importlib.import_module("oasis_msgs.srv")
    else:
        oasis_msgs_module = _make_package("oasis_msgs")
        oasis_msgs_msg_module = ModuleType("oasis_msgs.msg")
        oasis_msgs_srv_module = ModuleType("oasis_msgs.srv")

    _set_module_attr(oasis_msgs_msg_module, "AhrsStatus", _AhrsStatus)
    _set_module_attr(oasis_msgs_msg_module, "AnalogReading", _AnalogReading)
    _set_module_attr(oasis_msgs_msg_module, "AnalogReadings", _AnalogReadings)
    _set_module_attr(oasis_msgs_msg_module, "AVRConstants", _AVRConstants)
    _set_module_attr(oasis_msgs_msg_module, "DigitalReading", _DigitalReading)
    _set_module_attr(oasis_msgs_msg_module, "DigitalWriteCommand", _DigitalWriteCommand)
    _set_module_attr(oasis_msgs_msg_module, "PWMWriteCommand", _PWMWriteCommand)
    _set_module_attr(oasis_msgs_msg_module, "SerialDevice", _SerialDevice)
    _set_module_attr(oasis_msgs_msg_module, "SerialDeviceScan", _SerialDeviceScan)
    _set_module_attr(oasis_msgs_msg_module, "UPSStatus", _UPSStatus)
    _set_module_attr(oasis_msgs_msg_module, "UsbDevice", _UsbDevice)
    _set_module_attr(oasis_msgs_srv_module, "SetAnalogMode", _SetAnalogMode)
    _set_module_attr(oasis_msgs_srv_module, "SetDigitalMode", _SetDigitalMode)
    _set_module_attr(oasis_msgs_srv_module, "UPSCommand", _UPSCommand)
    _set_module_attr(oasis_msgs_module, "msg", oasis_msgs_msg_module)
    _set_module_attr(oasis_msgs_module, "srv", oasis_msgs_srv_module)
    _register_module("oasis_msgs", oasis_msgs_module)
    _register_module("oasis_msgs.msg", oasis_msgs_msg_module)
    _register_module("oasis_msgs.srv", oasis_msgs_srv_module)


def _install_launch_stub() -> None:
    if _module_exists("launch.launch_description"):
        return

    launch_module: ModuleType = _make_package("launch")
    launch_description_module: ModuleType = ModuleType("launch.launch_description")
    _set_module_attr(
        launch_description_module,
        "LaunchDescription",
        _LaunchDescription,
    )
    _set_module_attr(launch_module, "launch_description", launch_description_module)
    _register_module("launch", launch_module)
    _register_module("launch.launch_description", launch_description_module)


def _install_launch_ros_stub() -> None:
    if _module_exists("launch_ros.actions"):
        return

    launch_ros_module: ModuleType = _make_package("launch_ros")
    launch_ros_actions_module: ModuleType = ModuleType("launch_ros.actions")
    _set_module_attr(launch_ros_actions_module, "Node", _LaunchNode)
    _set_module_attr(launch_ros_module, "actions", launch_ros_actions_module)
    _register_module("launch_ros", launch_ros_module)
    _register_module("launch_ros.actions", launch_ros_actions_module)


def _install_rclpy_stub() -> None:
    if _module_exists("rclpy"):
        return

    rclpy_module: ModuleType = _make_package("rclpy")
    rclpy_client_module: ModuleType = ModuleType("rclpy.client")
    rclpy_logging_module: ModuleType = ModuleType("rclpy.logging")
    rclpy_node_module: ModuleType = ModuleType("rclpy.node")
    rclpy_publisher_module: ModuleType = ModuleType("rclpy.publisher")
    rclpy_qos_module: ModuleType = ModuleType("rclpy.qos")
    rclpy_service_module: ModuleType = ModuleType("rclpy.service")
    rclpy_subscription_module: ModuleType = ModuleType("rclpy.subscription")
    rclpy_task_module: ModuleType = ModuleType("rclpy.task")
    rclpy_time_module: ModuleType = ModuleType("rclpy.time")
    rclpy_timer_module: ModuleType = ModuleType("rclpy.timer")

    def _init(*_: Any, **__: Any) -> None:
        return None

    def _shutdown(*_: Any, **__: Any) -> None:
        return None

    def _ok() -> bool:
        return False

    def _spin_until_future_complete(node: object, future: _Future) -> None:
        del node, future
        return None

    _set_module_attr(rclpy_module, "init", _init)
    _set_module_attr(
        rclpy_module,
        "spin_until_future_complete",
        _spin_until_future_complete,
    )
    _set_module_attr(rclpy_module, "shutdown", _shutdown)
    _set_module_attr(rclpy_module, "ok", _ok)
    _set_module_attr(rclpy_client_module, "Client", _Client)
    _set_module_attr(rclpy_logging_module, "RcutilsLogger", _Logger)
    _set_module_attr(rclpy_node_module, "Node", _Node)
    _set_module_attr(rclpy_publisher_module, "Publisher", _Publisher)
    _set_module_attr(rclpy_qos_module, "QoSHistoryPolicy", _QoSHistoryPolicy)
    _set_module_attr(rclpy_qos_module, "QoSProfile", _QoSProfile)
    _set_module_attr(rclpy_qos_module, "QoSPresetProfiles", _QoSPresetProfiles)
    _set_module_attr(
        rclpy_qos_module,
        "QoSReliabilityPolicy",
        _QoSReliabilityPolicy,
    )
    _set_module_attr(rclpy_service_module, "Service", _Service)
    _set_module_attr(rclpy_subscription_module, "Subscription", _Subscription)
    _set_module_attr(rclpy_task_module, "Future", _Future)
    _set_module_attr(rclpy_time_module, "Time", _TimeHandle)
    _set_module_attr(rclpy_timer_module, "Timer", _Timer)

    _set_module_attr(rclpy_module, "client", rclpy_client_module)
    _set_module_attr(rclpy_module, "logging", rclpy_logging_module)
    _set_module_attr(rclpy_module, "node", rclpy_node_module)
    _set_module_attr(rclpy_module, "publisher", rclpy_publisher_module)
    _set_module_attr(rclpy_module, "qos", rclpy_qos_module)
    _set_module_attr(rclpy_module, "service", rclpy_service_module)
    _set_module_attr(rclpy_module, "subscription", rclpy_subscription_module)
    _set_module_attr(rclpy_module, "task", rclpy_task_module)
    _set_module_attr(rclpy_module, "time", rclpy_time_module)
    _set_module_attr(rclpy_module, "timer", rclpy_timer_module)

    _register_module("rclpy", rclpy_module)
    _register_module("rclpy.client", rclpy_client_module)
    _register_module("rclpy.logging", rclpy_logging_module)
    _register_module("rclpy.node", rclpy_node_module)
    _register_module("rclpy.publisher", rclpy_publisher_module)
    _register_module("rclpy.qos", rclpy_qos_module)
    _register_module("rclpy.service", rclpy_service_module)
    _register_module("rclpy.subscription", rclpy_subscription_module)
    _register_module("rclpy.task", rclpy_task_module)
    _register_module("rclpy.time", rclpy_time_module)
    _register_module("rclpy.timer", rclpy_timer_module)
