<!--
  Copyright (C) 2026 Garrett Brown
  SPDX-License-Identifier: Apache-2.0
-->

# ACS37800 hardware smoke test

This procedure is intentionally manual and is not part of CI.

1. Confirm both SparkFun Qwiic Power Meter boards are installed on the expected
   PCA9548 channels and that an independent voltmeter and current reference are
   available.
2. Launch the Airlab drivers with the power-meter component logger at debug
   level. Confirm each startup log reports `crs_sns 4`. This value is decoded
   from register `0x1B` bits 21:19 and checked against `expected_crs_sns`; it
   does not select the board's physical current range. Each sample log includes
   the adapter, mux channel, raw `0x2A`, `0x2C`, and `0x2D` values, and their
   decoded SI values.
3. With no load, compare the published voltage and current to the independent
   instruments. Apply a known load in the forward direction, then reverse the
   current direction and confirm that current and power become negative.
4. Confirm the power-meter topics exist, then inspect one message from each
   detected channel:

   ```sh
   ros2 topic list | grep power_meter
   ros2 topic echo --once /oasis/airlab/power_meter_0
   ros2 topic echo --once /oasis/airlab/power_meter_1
   ```

5. Confirm both messages use `STATUS_OK`, share plausible physical values, and
   change with load. Disconnect one meter and verify `STATUS_ERROR` followed by
   `STATUS_DISCONNECTED`; reconnect it and verify recovery to `STATUS_OK`.

The message timestamps are host acquisition timestamps because the ACS37800
does not provide hardware timestamps. Registers `0x2A`, `0x2C`, and `0x2D` are
read sequentially because Rev. 4 documents no coherent multi-register snapshot
mechanism.

The SparkFun SEN-29259 is configured explicitly as a 30 A board through
`current_sense_range_amps`. Current decoding remains the signed raw current
code divided by 27500 and multiplied by that physical range; `crs_sns` is only
a startup consistency check.
