#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from typing import List
from typing import Optional

import psutil
import rclpy.publisher
import std_msgs.msg
from builtin_interfaces.msg import Time as TimeMsg

from oasis_drivers.system.network_utils import NetworkUtils
from oasis_drivers.system.system_types import Battery
from oasis_drivers.system.system_types import DiskPartition
from oasis_drivers.system.system_types import NetworkAddress
from oasis_drivers.system.system_types import NetworkInterface
from oasis_msgs.msg import DiskPartition as DiskPartitionMsg
from oasis_msgs.msg import NetworkAddress as NetworkAddressMsg
from oasis_msgs.msg import NetworkInterface as NetworkInterfaceMsg
from oasis_msgs.msg import SystemTelemetry as SystemTelemetryMsg


class SystemMonitor:
    """
    Utility class for interacting with psutil, the cross-platform library for
    retrieving process and system information.

    Reports the following data:

      - CPU utilization
      - CPU frequency
      - CPU temperature
      - Memory utilization
      - Disk utilization
      - Network I/O

    Reports the following state:

      - CPU counts
      - CPU frequencies
      - Memory configuration
      - Disk configuration
      - Network addresses
    """

    @classmethod
    def read_psutil(
        cls, telemetry_pub: rclpy.publisher.Publisher, timestamp: TimeMsg, frame_id: str
    ) -> None:
        # Read information from psutil
        bootstrap_time = float(psutil.boot_time())
        cpu_utilization = float(psutil.cpu_percent())
        cpu_temperature = cls._read_cpu_temperature()
        cpu_frequency_ghz = cls._read_cpu_frequency_ghz()
        cpu_logical_core_count = int(psutil.cpu_count(logical=True))

        #
        # psutil.cpu_count() can return None on RPi because /proc/cpuinfo has a
        # different format and psutil doesn't want to play guessing games.
        #
        # See:
        #
        #   https://github.com/giampaolo/psutil/issues/1078
        #
        try:
            cpu_physical_core_count = int(psutil.cpu_count(logical=False))
        except TypeError:
            cpu_physical_core_count = cpu_logical_core_count

        memory_utilization = float(psutil.virtual_memory().percent)
        disk_partitions = cls._get_disk_partitions()
        network_interfaces = cls._get_network_interfaces()
        battery = cls._get_battery()

        # Timestamp in ROS header
        header = std_msgs.msg.Header()
        header.stamp = timestamp
        header.frame_id = frame_id

        # Create the message
        telemetry_msg = SystemTelemetryMsg()

        # Populate the message
        telemetry_msg.header = header
        telemetry_msg.bootstrap_time = bootstrap_time
        telemetry_msg.cpu_utilization = cpu_utilization
        if cpu_temperature is not None:
            telemetry_msg.cpu_temperature = cpu_temperature
        if cpu_frequency_ghz is not None:
            telemetry_msg.cpu_frequency_ghz = cpu_frequency_ghz
        telemetry_msg.cpu_physical_core_count = cpu_physical_core_count
        telemetry_msg.cpu_logical_core_count = cpu_logical_core_count
        telemetry_msg.memory_utilization = memory_utilization
        for partition in disk_partitions:
            disk_partition_msg = DiskPartitionMsg()
            disk_partition_msg.device_name = partition.device_name
            disk_partition_msg.filesystem = partition.filesystem
            disk_partition_msg.mount_point = partition.mount_point
            disk_partition_msg.mount_options = partition.mount_options
            disk_partition_msg.disk_total = partition.disk_total
            disk_partition_msg.disk_used = partition.disk_used
            disk_partition_msg.disk_free = partition.disk_free
            disk_partition_msg.disk_utilization = partition.disk_utilization
            telemetry_msg.disk_partitions.append(disk_partition_msg)
        for interface in network_interfaces:
            network_interface_msg = NetworkInterfaceMsg()
            network_interface_msg.name = interface.name
            network_interface_msg.provider = interface.provider
            network_interface_msg.bytes_sent = interface.bytes_sent
            network_interface_msg.bytes_received = interface.bytes_received
            for address in interface.addresses:
                network_address_msg = NetworkAddressMsg()
                network_address_msg.family = address.family
                network_address_msg.address = address.address
                network_address_msg.netmask = address.netmask
                network_interface_msg.addresses.append(network_address_msg)
            telemetry_msg.network_interfaces.append(network_interface_msg)
        if battery is not None:
            telemetry_msg.has_battery = True
            telemetry_msg.battery_percent = battery.battery_percent
            telemetry_msg.battery_remaining_mins = battery.battery_remaining_time
            telemetry_msg.power_plugged = battery.power_plugged
        else:
            telemetry_msg.has_battery = False

        # Publish the message
        telemetry_pub.publish(telemetry_msg)

    @staticmethod
    def _read_cpu_temperature() -> Optional[float]:
        for sensor, shwtemps in psutil.sensors_temperatures().items():
            max_temp: float = float(
                max([float(shwtemp.current) for shwtemp in shwtemps])
            )

            # Check Intel digital thermal sensor
            if sensor == "coretemp":
                # Return the max temp
                return max_temp

            # Check for Raspberry Pi CPU temperature sensor
            if sensor == "cpu_thermal":
                return max_temp

            # Check AMD K8 thermal sensor
            if sensor == "k8temp":
                # Return the max temp
                return max_temp

        return None

    @staticmethod
    def _read_cpu_frequency_ghz() -> Optional[float]:
        freq = psutil.cpu_freq()

        if freq:
            return float(freq.current) / 1000.0

        return None

    @staticmethod
    def _get_disk_partitions() -> List[DiskPartition]:
        disk_partitions = []

        for partition in psutil.disk_partitions():
            mount_point = str(partition.mountpoint)
            disk_usage = psutil.disk_usage(mount_point)

            # Skip snap mounts
            if mount_point.startswith("/snap"):
                continue

            disk_partitions.append(
                DiskPartition(
                    device_name=str(partition.device),
                    filesystem=str(partition.fstype),
                    mount_point=mount_point,
                    mount_options=str(partition.opts),
                    disk_total=int(disk_usage.total),
                    disk_used=int(disk_usage.used),
                    disk_free=int(disk_usage.free),
                    disk_utilization=float(disk_usage.percent),
                )
            )

        return disk_partitions

    @classmethod
    def _get_network_interfaces(cls) -> List[NetworkInterface]:
        net_io_counters = psutil.net_io_counters(pernic=True)
        net_addresses = psutil.net_if_addrs()

        interfaces = [
            NetworkInterface(
                name=interface,
                provider=cls._get_interface_provider(interface),
                bytes_sent=int(netio.bytes_sent),
                bytes_received=int(netio.bytes_recv),
                addresses=[
                    NetworkAddress(
                        family=NetworkUtils.get_network_address_family(snicaddr.family),
                        address=str(snicaddr.address),
                        netmask=str(snicaddr.netmask) if snicaddr.netmask else "",
                    )
                    for snicaddr in net_addresses.get(interface, [])
                ],
            )
            for interface, netio in net_io_counters.items()
            if interface != "lo"
        ]

        return interfaces

    @staticmethod
    def _get_interface_provider(interface_name: str) -> str:
        # TODO
        return ""

    @staticmethod
    def _get_battery() -> Optional[Battery]:
        battery = psutil.sensors_battery()

        if battery:
            return Battery(
                battery_percent=float(battery.percent),
                battery_remaining_time=float(battery.secsleft) / 60.0,
                power_plugged=bool(battery.power_plugged),
            )

        return None
