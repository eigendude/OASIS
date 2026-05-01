################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from collections.abc import Sequence

from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Header as HeaderMsg

from oasis_drivers.mcu.mcu_readings import AnalogReadingSample
from oasis_drivers.mcu.mcu_readings import DigitalReadingSample
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import AnalogReadings as AnalogReadingsMsg
from oasis_msgs.msg import AVRConstants as AVRConstantsMsg
from oasis_msgs.msg import DigitalReading as DigitalReadingMsg


def make_analog_reading_msg(
    stamp: TimeMsg, sample: AnalogReadingSample
) -> AnalogReadingMsg:
    """Build a ROS analog reading message."""

    msg: AnalogReadingMsg = AnalogReadingMsg()
    msg.header = _make_header(stamp)
    msg.analog_pin = sample.analog_pin
    msg.reference_voltage = sample.reference_voltage
    msg.analog_value = sample.analog_value

    return msg


def make_digital_reading_msg(
    stamp: TimeMsg, sample: DigitalReadingSample
) -> DigitalReadingMsg:
    """Build a ROS digital reading message."""

    msg: DigitalReadingMsg = DigitalReadingMsg()
    msg.header = _make_header(stamp)
    msg.digital_pin = sample.digital_pin
    msg.digital_value = (
        AVRConstantsMsg.HIGH if sample.digital_value else AVRConstantsMsg.LOW
    )

    return msg


def make_analog_readings_msg(
    stamp: TimeMsg, samples: Sequence[AnalogReadingSample]
) -> AnalogReadingsMsg:
    """Build a ROS analog readings batch with a shared header stamp."""

    msg: AnalogReadingsMsg = AnalogReadingsMsg()
    msg.header = _make_header(stamp)
    msg.readings = [make_analog_reading_msg(stamp, sample) for sample in samples]

    return msg


def _make_header(stamp: TimeMsg) -> HeaderMsg:
    header: HeaderMsg = HeaderMsg()
    header.stamp = stamp
    header.frame_id = ""

    return header
