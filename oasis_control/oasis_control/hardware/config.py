################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from dataclasses import dataclass
from enum import Enum


class MCUManagerImplementation(Enum):
    """Control manager used for an installed microcontroller"""

    # LEGO train conductor manager
    CONDUCTOR = "conductor"

    # LEGO train engineer manager
    ENGINEER = "engineer"


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

    # Zone providing camera scenes to a conductor manager
    camera_scene_zone: str | None = None

    # Resolution path segment for conductor camera scenes, or none if omitted
    camera_scene_resolution: str | None = None

    # Host providing Wake-on-LAN services to a conductor manager
    wol_server_id: str | None = None

    # Whether positive motor voltage commands use reversed physical polarity
    motor_voltage_reversed: bool = False


@dataclass(frozen=True)
class HostHardwareConfig:
    """Complete control-facing hardware configuration for one OASIS host"""

    # Whether the host supplies AHRS data for forward-speed estimation
    enable_ahrs_speedometer: bool = False

    # Whether an OLED is installed in the Millennium Falcon cockpit
    enable_cockpit_visualizer: bool = False

    # Whether an OLED is installed and should receive visualization output
    enable_oled_visualizer: bool = False

    # Whether the host should expose Wake-on-LAN control for managed devices
    enable_wol_server: bool = False

    # Optional control manager for the host-attached microcontroller
    mcu_manager: MCUManagerConfig | None = None
