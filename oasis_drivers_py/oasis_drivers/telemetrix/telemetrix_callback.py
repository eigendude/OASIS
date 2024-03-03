################################################################################
#
#  Copyright (C) 2022-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import abc
from datetime import datetime


class TelemetrixCallback:
    @abc.abstractmethod
    def on_air_quality(
        self,
        timestamp: datetime,
        i2c_port: int,
        i2c_address: int,
        co2_ppb: int,
        tvoc_ppb: int,
    ) -> None:
        """
        Handle reports of an air quality sensor with CO2 and TVOC modalities.
        """
        pass

    @abc.abstractmethod
    def on_analog_reading(
        self,
        timestamp: datetime,
        analog_pin: int,
        analog_value: float,
        reference_voltage: float,
    ) -> None:
        pass

    @abc.abstractmethod
    def on_cpu_fan_rpm(self, timestamp: datetime, digital_pin: int, rpm: int) -> None:
        pass

    @abc.abstractmethod
    def on_digital_reading(
        self, timestamp: datetime, digital_pin: int, digital_value: bool
    ) -> None:
        pass

    @abc.abstractmethod
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
        """
        Handle reports of a 6-axis IMU.
        """
        pass

    @abc.abstractmethod
    def on_memory_data(
        self,
        total_ram: int,
        static_data_size: int,
        heap_size: int,
        stack_size: int,
        free_ram: int,
        free_heap: int,
    ) -> None:
        pass

    @abc.abstractmethod
    def on_string_data(self, data: str) -> None:
        pass
