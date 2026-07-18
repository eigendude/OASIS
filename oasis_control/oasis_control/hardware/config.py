################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class MCUManagerImplementation(Enum):
    """Control manager used for an installed microcontroller"""

    # General-purpose manager for an MCU without specialized control behavior
    STANDARD = "standard"

    # Manager for an MCU that exposes pulse-width modulation outputs
    PWM = "pwm"

    # LEGO train conductor manager
    CONDUCTOR = "conductor"


@dataclass(frozen=True)
class MCUManagerConfig:
    """Control configuration for one host-attached microcontroller"""

    # Logical MCU name used to select the manager executable and ROS topics
    node_name: str

    # Manager implementation appropriate for the attached hardware
    implementation: MCUManagerImplementation

    # Host providing conductor state to a PWM manager
    conductor_host: str | None = None

    # Host providing peripheral input to a conductor manager
    input_provider: str | None = None

    # Host providing Wake-on-LAN services to a conductor manager
    wol_server_id: str | None = None

    # Whether positive motor voltage commands use reversed physical polarity
    motor_voltage_reversed: bool = False


@dataclass(frozen=True)
class HostHardwareConfig:
    """Complete control-facing hardware configuration for one OASIS host"""

    # Whether the host supplies AHRS data for forward-speed estimation
    enable_ahrs_speedometer: bool = False

    # Optional control manager for the host-attached microcontroller
    mcu_manager: MCUManagerConfig | None = None

    # Whether the host should expose Wake-on-LAN control for managed devices
    enable_wol_server: bool = False
