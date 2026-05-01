################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import math
import threading
from typing import Any
from typing import Optional

import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.service
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import BatteryState as BatteryStateMsg

from oasis_drivers.ups.ups_server import UpsServer
from oasis_msgs.msg import UPSStatus as UPSStatusMsg
from oasis_msgs.srv import UPSCommand as UPSCommandSvc


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ups_server"

UPS_STATUS_PUBLISHER: str = "ups_status"
BATTERY_STATE_PUBLISHER: str = "battery"

UPS_COMMAND_SERVICE: str = "ups_command"

BATTERY_LOCATION: str = "ups"

NUT_STATUS_ON_LINE: str = "ON_LINE"
NUT_STATUS_ON_BATTERY: str = "ON_BATTERY"
NUT_STATUS_LOW_BATTERY: str = "LOW_BATTERY"
NUT_STATUS_HIGH_BATTERY: str = "HIGH_BATTERY"
NUT_STATUS_REPLACE_BATTERY: str = "REPLACE_BATTERY"
NUT_STATUS_CHARGING: str = "CHARGING"
NUT_STATUS_DISCHARGING: str = "DISCHARGING"
NUT_STATUS_OFF: str = "OFF"
NUT_STATUS_OVERLOAD: str = "OVERLOAD"

NUT_STATUS_TOKEN_ALIASES: dict[str, str] = {
    "OL": NUT_STATUS_ON_LINE,
    "OB": NUT_STATUS_ON_BATTERY,
    "LB": NUT_STATUS_LOW_BATTERY,
    "HB": NUT_STATUS_HIGH_BATTERY,
    "RB": NUT_STATUS_REPLACE_BATTERY,
    "CHRG": NUT_STATUS_CHARGING,
    "DISCHRG": NUT_STATUS_DISCHARGING,
    "OVER": NUT_STATUS_OVERLOAD,
}

# Unitless fraction treated as full to tolerate rounded 100% charge reports
FULL_PERCENTAGE_THRESHOLD: float = 0.995


################################################################################
# Timing parameters
################################################################################


# The UPS server will poll the UPS every second when connected, and every 10s
# when disconnected. This is a compromise between responsiveness and CPU usage.
# Polling interval in seconds while a UPS is connected
UPS_POLLING_INTERVAL_CONNECTED: float = 1.0

# Polling interval in seconds while no UPS is detected
UPS_POLLING_INTERVAL_DISCONNECTED: float = 10.0


################################################################################
# ROS node
################################################################################


class UpsServerNode(rclpy.node.Node):
    def __init__(self, start_thread: bool = True) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # ROS publishers
        self._ups_status_publisher: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=UPSStatusMsg,
            topic=UPS_STATUS_PUBLISHER,
            qos_profile=rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self._battery_state_publisher: rclpy.publisher.Publisher[BatteryStateMsg] = (
            self.create_publisher(
                msg_type=BatteryStateMsg,
                topic=BATTERY_STATE_PUBLISHER,
                qos_profile=_make_state_qos_profile(),
            )
        )

        # ROS services
        self._ups_control_service: rclpy.service.Service = self.create_service(
            srv_type=UPSCommandSvc,
            srv_name=UPS_COMMAND_SERVICE,
            callback=self._handle_ups_command,
        )

        # Threading parameters
        self._exit_event: threading.Event = threading.Event()
        self._thread: threading.Thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
        )
        self._thread_started: bool = False

        self.get_logger().info("UPS server initialized")

        # Start the monitoring thread
        if start_thread:
            self._thread.start()
            self._thread_started = True

    def stop(self) -> None:
        self._exit_event.set()
        if self._thread_started:
            self._thread.join()

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        self.destroy_node()

        self.get_logger().info("UPS server deinitialized")

    def _monitor_loop(self) -> None:
        # Loop state
        ups_connected: bool = False
        error_logged: bool = False
        msg: Optional[UPSStatusMsg] = None

        while rclpy.ok():
            # Enter idle mode if the UPS is not connected
            if not ups_connected:
                # Idle mode: check every 10s for a UPS
                msg = UpsServer.read_status(self.get_logger())

                # If we get a valid message, the UPS is connected
                if msg is not None:
                    # Update state
                    ups_connected = True
                    error_logged = False

                    # Log success message
                    self.get_logger().info(
                        f"UPS connected, entering active mode at {UPS_POLLING_INTERVAL_CONNECTED}s intervals"
                    )

                    # Publish message
                    self._publish_status_messages(msg)

                    # Fall through into active mode
                    pass
                else:
                    if not error_logged:
                        # Update state
                        error_logged = True

                        # Log error message
                        self.get_logger().error(
                            f"No UPS detected, retrying every {UPS_POLLING_INTERVAL_DISCONNECTED}s"
                        )

                    # Sleep before retrying in idle mode
                    if self._exit_event.wait(timeout=UPS_POLLING_INTERVAL_DISCONNECTED):
                        # Exit signal received, exit from idle mode
                        break

                    # No UPS detected, re-enter idle mode
                    continue

            # Active mode: UPS is connected, poll and publish regularly
            msg = UpsServer.read_status(self.get_logger())

            # If we get a invalid message, the UPS is disconnected
            if msg is None:
                # Update state
                ups_connected = False

                # Log error message
                self.get_logger().error("UPS disconnected, returning to idle mode")

                # Loop back into idle mode
                continue

            # Publish message
            self._publish_status_messages(msg)

            # Wait up to 1s for exit signal, then loop in active mode again
            if self._exit_event.wait(timeout=UPS_POLLING_INTERVAL_CONNECTED):
                # Exit signal received, exit from active mode
                break

    def _handle_ups_command(
        self, request: UPSCommandSvc.Request, response: UPSCommandSvc.Response
    ) -> UPSCommandSvc.Response:
        success: bool = UpsServer.send_command(
            request.command, request.delay, self.get_logger()
        )

        # Log result
        if success:
            self.get_logger().info(
                f"UPS command '{request.command}' executed successfully"
            )
        else:
            self.get_logger().error(f"UPS command '{request.command}' failed")

        # We can enhance this to include success/failure fields if needed
        return response

    def _publish_status_messages(self, msg: UPSStatusMsg) -> None:
        self._ups_status_publisher.publish(msg)
        self._battery_state_publisher.publish(
            _make_battery_state_msg(
                msg=msg,
                stamp=self.get_clock().now().to_msg(),
            )
        )


def _make_state_qos_profile() -> rclpy.qos.QoSProfile:
    return rclpy.qos.QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _make_battery_state_msg(
    msg: UPSStatusMsg,
    stamp: TimeMsg,
) -> BatteryStateMsg:
    battery_msg: BatteryStateMsg = BatteryStateMsg()
    battery_msg.header.stamp = stamp
    battery_msg.voltage = _battery_voltage(msg)
    battery_msg.temperature = _finite_float_or_nan(msg.battery_temperature)
    battery_msg.current = _battery_current(msg)
    battery_msg.charge = math.nan
    battery_msg.capacity = math.nan
    battery_msg.design_capacity = math.nan
    battery_msg.percentage = _battery_percentage(msg.battery_charge)
    battery_msg.power_supply_status = _power_supply_status(
        status=msg.status,
        percentage=battery_msg.percentage,
    )
    battery_msg.power_supply_health = _power_supply_health(msg.status)
    battery_msg.power_supply_technology = _power_supply_technology(msg.battery_type)
    battery_msg.present = True
    battery_msg.cell_voltage = []
    battery_msg.cell_temperature = []
    battery_msg.location = BATTERY_LOCATION
    battery_msg.serial_number = msg.serial_number

    return battery_msg


def _battery_voltage(msg: UPSStatusMsg) -> float:
    battery_voltage: float = _finite_float_or_nan(msg.battery_voltage)
    if not math.isnan(battery_voltage):
        return battery_voltage

    output_voltage: float = _finite_float_or_nan(msg.output_voltage)
    if not math.isnan(output_voltage):
        return output_voltage

    return _finite_float_or_nan(msg.input_voltage)


def _battery_current(msg: UPSStatusMsg) -> float:
    status_tokens: set[str] = _status_tokens(msg.status)
    battery_current: float = _finite_float_or_nan(msg.battery_current)

    if not math.isnan(battery_current):
        if _is_discharging(status_tokens):
            return -abs(battery_current)

        if _is_charging(status_tokens):
            return abs(battery_current)

        return battery_current

    if not _is_discharging(status_tokens):
        return math.nan

    load_watts: float = _finite_float_or_nan(msg.load)
    battery_voltage: float = _finite_float_or_nan(msg.battery_voltage)
    if math.isnan(load_watts) or math.isnan(battery_voltage) or battery_voltage <= 0.0:
        return math.nan

    # Approximate DC current in A from P = V * I, ignoring inverter losses
    return -abs(load_watts / battery_voltage)


def _battery_percentage(charge_percent: Any) -> float:
    charge: float = _finite_float_or_nan(charge_percent)
    if math.isnan(charge):
        return math.nan

    # Convert percent on [0, 100] to BatteryState's unitless [0, 1] range
    return min(max(charge, 0.0), 100.0) / 100.0


def _power_supply_status(status: str, percentage: float) -> int:
    status_tokens: set[str] = _status_tokens(status)

    if _is_charging(status_tokens):
        return BatteryStateMsg.POWER_SUPPLY_STATUS_CHARGING

    if _is_discharging(status_tokens):
        return BatteryStateMsg.POWER_SUPPLY_STATUS_DISCHARGING

    if NUT_STATUS_ON_LINE in status_tokens:
        if not math.isnan(percentage) and percentage >= FULL_PERCENTAGE_THRESHOLD:
            return BatteryStateMsg.POWER_SUPPLY_STATUS_FULL

        return BatteryStateMsg.POWER_SUPPLY_STATUS_NOT_CHARGING

    if NUT_STATUS_OFF in status_tokens:
        return BatteryStateMsg.POWER_SUPPLY_STATUS_NOT_CHARGING

    return BatteryStateMsg.POWER_SUPPLY_STATUS_UNKNOWN


def _power_supply_health(status: str) -> int:
    status_tokens: set[str] = _status_tokens(status)

    if not status_tokens:
        return BatteryStateMsg.POWER_SUPPLY_HEALTH_UNKNOWN

    if NUT_STATUS_REPLACE_BATTERY in status_tokens:
        return BatteryStateMsg.POWER_SUPPLY_HEALTH_DEAD

    if NUT_STATUS_HIGH_BATTERY in status_tokens:
        return BatteryStateMsg.POWER_SUPPLY_HEALTH_OVERVOLTAGE

    if NUT_STATUS_OVERLOAD in status_tokens:
        return BatteryStateMsg.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE

    return BatteryStateMsg.POWER_SUPPLY_HEALTH_GOOD


def _power_supply_technology(battery_type: str) -> int:
    normalized_type: str = _normalize_battery_type(battery_type)

    if normalized_type in {"liion", "lion", "lithiumion"}:
        return BatteryStateMsg.POWER_SUPPLY_TECHNOLOGY_LION

    if normalized_type in {"lipo", "lithiumpolymer"}:
        return BatteryStateMsg.POWER_SUPPLY_TECHNOLOGY_LIPO

    if normalized_type in {"life", "lifepo4", "lithiumironphosphate"}:
        return BatteryStateMsg.POWER_SUPPLY_TECHNOLOGY_LIFE

    if normalized_type == "nimh":
        return BatteryStateMsg.POWER_SUPPLY_TECHNOLOGY_NIMH

    if normalized_type == "nicd":
        return BatteryStateMsg.POWER_SUPPLY_TECHNOLOGY_NICD

    if normalized_type in {"limn", "lithiummanganese"}:
        return BatteryStateMsg.POWER_SUPPLY_TECHNOLOGY_LIMN

    return BatteryStateMsg.POWER_SUPPLY_TECHNOLOGY_UNKNOWN


def _normalize_battery_type(battery_type: str) -> str:
    return (
        battery_type.strip().lower().replace("-", "").replace("_", "").replace(" ", "")
    )


def _is_charging(status_tokens: set[str]) -> bool:
    return NUT_STATUS_CHARGING in status_tokens


def _is_discharging(status_tokens: set[str]) -> bool:
    return (
        NUT_STATUS_DISCHARGING in status_tokens
        or NUT_STATUS_ON_BATTERY in status_tokens
    )


def _status_tokens(status: str) -> set[str]:
    return {_normalize_status_token(token) for token in status.split() if token.strip()}


def _normalize_status_token(token: str) -> str:
    raw_token: str = token.strip().upper()
    return NUT_STATUS_TOKEN_ALIASES.get(raw_token, raw_token)


def _finite_float_or_nan(value: Any) -> float:
    try:
        finite_value: float = float(value)
    except (TypeError, ValueError):
        return math.nan

    if not math.isfinite(finite_value):
        return math.nan

    return finite_value
