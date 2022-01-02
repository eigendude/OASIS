################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import asyncio
import threading
from concurrent.futures import Future
from datetime import datetime
from typing import Any
from typing import Awaitable
from typing import List
from typing import Tuple

from pymata_express import pymata_express
from pymata_express.private_constants import PrivateConstants

from oasis_drivers.firmata.firmata_callback import FirmataCallback
from oasis_drivers.firmata.firmata_types import AnalogMode
from oasis_drivers.firmata.firmata_types import DigitalMode


class FirmataBridge:
    """
    Bridge to Firmata server running on an AVR processor.
    """

    # AVR parameters
    COM_PORT = "/dev/ttyACM0"  # TODO
    BAUD_RATE = 115200
    ARDUINO_INSTANCE_ID = 1  # TODO
    ARDUINO_WAIT_SECS = 2  # TODO

    # Firmata callback data indices
    CB_PIN_MODE = 0
    CB_PIN = 1
    CB_VALUE = 2
    CB_TIME = 3

    # ADC parameters
    ANALOG_MAX: int = (1 << 10) - 1  # Max 10-bit analog reading is 1023
    ANALOG_REFERENCE: float = 5.0  # Volts (TODO: Where to get this number?)
    PWM_MAX: int = 255  # TODO: Max PWM value?

    def __init__(self, callback: FirmataCallback) -> None:
        # Construction parameters
        self._callback = callback

        # Initialize asyncio event loop for running bridge in a new thread
        self._loop: asyncio.AbstractEventLoop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_thread)

        # Instantiate pymata-express
        self._board = pymata_express.PymataExpress(
            com_port=self.COM_PORT,
            baud_rate=self.BAUD_RATE,
            arduino_instance_id=self.ARDUINO_INSTANCE_ID,
            arduino_wait=self.ARDUINO_WAIT_SECS,
            autostart=False,
            loop=self._loop,
            shutdown_on_exception=True,
            close_loop_on_shutdown=False,
        )

        # Patch string-handling (pymata-express prints to stdout)
        self._board.command_dictionary[
            PrivateConstants.STRING_DATA
        ] = self._on_string_data

    def initialize(self) -> None:
        """Initialize the bridge and start communicating via Firmata"""
        self._thread.start()

        try:
            asyncio.run_coroutine_threadsafe(
                self._board.start_aio(), self._loop
            ).result()
        except Exception as e:
            self.deinitialize()
            raise e

    def deinitialize(self) -> None:
        """Stop communicating and deinitialize the bridge"""
        self._loop.stop()

        # Let's also cancel all running tasks
        pending = asyncio.all_tasks(loop=self._loop)
        for task in pending:
            task.cancel()

        self._thread.join()
        self._loop.close()

    def _run_thread(self) -> None:
        """Run the asyncio event loop in a thread to process Firmata coroutines"""
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def set_analog_mode(self, analog_pin: int, analog_mode: AnalogMode) -> None:
        """
        Set the analog mode of a specified analog pin.

        :param analog_pin: Analog pin number (ex. A2 is specified as 2)
        :param analog_mode: The new mode for the analog pin
        """
        coroutine: Awaitable[None]

        # Create coroutine
        if analog_mode == AnalogMode.DISABLED:
            coroutine = self._board.disable_analog_reporting(analog_pin)
        elif analog_mode == AnalogMode.INPUT:
            coroutine = self._board.set_pin_mode_analog_input(
                analog_pin, self._analog_read_callback, 0
            )
        else:
            raise ValueError(f"Invalid analog mode: {analog_mode}")

        # Dispatch to asyncio
        future: Future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)

        # Wait for completion
        future.result()

    def analog_read(self, analog_pin: int) -> Tuple[float, float, datetime]:
        """
        Retrieve the last data update for the specified analog pin.

        :param analog_pin: Analog pin number (ex. A2 is specified as 2)

        :return: Last reported analog value and a timestamp of the reading
        """
        # Create coroutine
        coroutine: Awaitable[Tuple[int, int]] = self._board.analog_read(analog_pin)

        # Dispatch to asyncio
        future: Future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)

        # Get result
        result: Tuple[int, int] = future.result()

        # Translate result
        analog_value: float = float(result[0]) / self.ANALOG_MAX
        reference_voltage: float = self.ANALOG_REFERENCE
        timestamp: datetime = self._get_timestamp(result[1])

        return analog_value, reference_voltage, timestamp

    def set_digital_mode(self, digital_pin: int, digital_mode: DigitalMode) -> None:
        """
        Set the digital mode of a specified digital pin.

        :param digital_pin: Digital pin number
        :param digital_mode: The new mode for the digital pin
        """
        coroutine: Awaitable[None]

        # Create coroutine
        if digital_mode == DigitalMode.DISABLED:
            coroutine = self._board.disable_digital_reporting(digital_pin)
        elif digital_mode == DigitalMode.INPUT:
            coroutine = self._board.set_pin_mode_digital_input(
                digital_pin, self._digital_read_callback
            )
        elif digital_mode == DigitalMode.INPUT_PULLUP:
            coroutine = self._board.set_pin_mode_digital_input_pullup(
                digital_pin, self._digital_read_callback
            )
        elif digital_mode == DigitalMode.OUTPUT:
            coroutine = self._board.set_pin_mode_digital_output(digital_pin)
        elif digital_mode == DigitalMode.PWM:
            coroutine = self._board.set_pin_mode_pwm_output(digital_pin)
        elif digital_mode == DigitalMode.SERVO:
            # TODO: Expose min_pulse and max_pulse parameters
            coroutine = self._board.set_pin_mode_servo(digital_pin)
        else:
            raise ValueError(f"Invalid digital mode: {digital_mode}")

        # Dispatch to asyncio
        future: Future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)

        # Wait for completion
        future.result()

    def digital_read(self, digital_pin: int) -> Tuple[bool, datetime]:
        """
        Retrieve the last data update for the specified digital pin.

        :param digital_pin: Digital pin number

        :return: Last reported digital value and a timestamp of the reading
        """
        # Create coroutine
        coroutine: Awaitable[Tuple[int, int]] = self._board.digital_read(digital_pin)

        # Dispatch to asyncio
        future: Future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)

        # Get result
        result: Tuple[int, int] = future.result()

        # Translate result
        digital_value: bool = bool(result[0])
        timestamp: datetime = self._get_timestamp(result[1])

        return digital_value, timestamp

    def digital_write(self, digital_pin: int, digital_value: bool) -> None:
        """
        Set the specified digital pin to the specified value.

        :param digital_pin: Digital pin number
        :param digital_value: Pin value (True for 1, False for 0)
        """
        # Translate parameters
        value_int = 1 if digital_value else 0

        # Create coroutine
        coroutine: Awaitable[None] = self._board.digital_write(digital_pin, value_int)

        # Dispatch to asyncio
        future: Future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)

        # Wait for completion
        future.result()

    def pwm_write(self, digital_pin: int, duty_cycle: float) -> None:
        """
        Retrieve the last data update for the specified PWM pin.

        :param digital_pin: Digital pin number
        :param duty_cycle: PWM duty cycle (0.0 - 1.0)
        """
        # Scale value to integer expected by Firmata
        value_int: int = int(duty_cycle * self.PWM_MAX)

        # Create coroutine
        coroutine: Awaitable[None] = self._board.pwm_write(digital_pin, value_int)

        # Dispatch to asyncio
        future: Future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)

        # Wait for completion
        future.result()

    def servo_write(self, digital_pin: int, position: float) -> None:
        """
        Retrieve the last data update for the specified PWM pin.

        :param digital_pin: Digital pin number
        :param position: The desired servo position, from 0.0 to 1.0
        """
        # Scale value to integer expected by Firmata
        value_int = int(position * self.PWM_MAX)  # TODO: Max value?

        # Create coroutine
        coroutine: Awaitable[None] = self._board.servo_write(digital_pin, value_int)

        # Dispatch to asyncio
        future: Future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)

        # Wait for completion
        future.result()

    async def _analog_read_callback(self, data: List[int]) -> None:
        """
        Report data changes on analog pins.

        This will print the pin number, its reported value and the date and
        time when the change occurred.

        :param data: [pin_mode, pin, current_reported_value, timestamp]
        """
        # Translate parameters
        analog_pin: int = data[self.CB_PIN]
        analog_value: float = float(data[self.CB_VALUE]) / self.ANALOG_MAX
        reference_voltage: float = self.ANALOG_REFERENCE
        timestamp: datetime = self._get_timestamp(data[self.CB_TIME])

        # TODO: Log local time
        timestamp_str: str = timestamp.strftime("%Y-%m-%d %H:%M:%S")
        print(
            f"Analog pin: {analog_pin}, value: {analog_value}, reference: {reference_voltage}V, timestamp: {timestamp_str}"
        )

        self._callback.on_analog_reading(
            timestamp, analog_pin, analog_value, reference_voltage
        )

    async def _digital_read_callback(self, data: List[Any]) -> None:
        """
        Report data changes on digital pins.

        This will print the pin number, its reported value and the date and
        time when the change occurred.

        :param data: [pin_mode, pin, current_reported_value, timestamp]
        """
        digital_pin: int = data[self.CB_PIN]
        digital_value: bool = True if data[self.CB_VALUE] else False
        timestamp: datetime = self._get_timestamp(data[self.CB_TIME])

        # TODO: Log local time
        timestamp_str: str = timestamp.strftime("%Y-%m-%d %H:%M:%S")
        print(
            f"Digital pin: {digital_pin}, value: {digital_value}, timestamp: {timestamp_str}"
        )

        self._callback.on_digital_reading(timestamp, digital_pin, digital_value)

    async def _on_string_data(self, data: List[int]) -> None:
        """
        Handle string messages

        This is the message handler for String data messages.

        :param data: The message
        """
        reply: str = ""

        data = data[1:-1]
        for x in data:
            reply_data: int = x
            if reply_data:
                reply += chr(reply_data)

        self._callback.on_string_data(reply)

    @staticmethod
    def _get_timestamp(unix_time: int) -> datetime:
        if unix_time == 0:
            # A timestamp of 0 means that no reading has been reported yet
            return datetime.fromtimestamp(unix_time)
        else:
            # TODO: Timezone awareness? pymata-express uses time.time(), which
            # is timezone-insensitive
            # return datetime.utcfromtimestamp(unix_time)
            return datetime.fromtimestamp(unix_time)
