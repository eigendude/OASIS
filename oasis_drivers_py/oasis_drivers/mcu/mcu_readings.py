################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from dataclasses import dataclass


@dataclass(frozen=True)
class AnalogReadingSample:
    """
    One normalized ADC sample from an MCU analog input.

    Attributes:
        analog_pin: Zero-based analog input number, for example A2 as 2.
        reference_voltage: ADC reference voltage in volts used to scale the
            normalized sample.
        analog_value: Normalized ADC value in the closed range [0.0, 1.0].
    """

    analog_pin: int
    reference_voltage: float
    analog_value: float


@dataclass(frozen=True)
class DigitalReadingSample:
    """
    One GPIO sample from an MCU digital input.

    Attributes:
        digital_pin: Digital IO pin number using board-native numbering.
        digital_value: True for logic-high, false for logic-low.
    """

    digital_pin: int
    digital_value: bool
