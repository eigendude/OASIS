/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// Client Command Related Defines and Support
////////////////////////////////////////////////////////////////////////////////

// Commands Sent By The Client

// Add commands retaining the sequential numbering. The order of commands here
// must be maintained in the command_table.
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define ANALOG_WRITE 3
#define MODIFY_REPORTING 4 // mode(all, analog, or digital), pin, enable or disable
#define GET_FIRMWARE_VERSION 5
#define ARE_U_THERE 6
#define SERVO_ATTACH 7
#define SERVO_WRITE 8
#define SERVO_DETACH 9
#define I2C_BEGIN 10
#define I2C_READ 11
#define I2C_WRITE 12
#define SONAR_NEW 13
#define DHT_NEW 14
#define STOP_ALL_REPORTS 15
#define SET_ANALOG_SCANNING_INTERVAL 16
#define ENABLE_ALL_REPORTS 17
#define RESET 18
#define SPI_INIT 19
#define SPI_WRITE_BLOCKING 20
#define SPI_READ_BLOCKING 21
#define SPI_SET_FORMAT 22
#define SPI_CS_CONTROL 23
#define ONE_WIRE_INIT 24
#define ONE_WIRE_RESET 25
#define ONE_WIRE_SELECT 26
#define ONE_WIRE_SKIP 27
#define ONE_WIRE_WRITE 28
#define ONE_WIRE_READ 29
#define ONE_WIRE_RESET_SEARCH 30
#define ONE_WIRE_SEARCH 31
#define ONE_WIRE_CRC8 32
#define SET_PIN_MODE_STEPPER 33
#define STEPPER_MOVE_TO 34
#define STEPPER_MOVE 35
#define STEPPER_RUN 36
#define STEPPER_RUN_SPEED 37
#define STEPPER_SET_MAX_SPEED 38
#define STEPPER_SET_ACCELERATION 39
#define STEPPER_SET_SPEED 40
#define STEPPER_SET_CURRENT_POSITION 41
#define STEPPER_RUN_SPEED_TO_POSITION 42
#define STEPPER_STOP 43
#define STEPPER_DISABLE_OUTPUTS 44
#define STEPPER_ENABLE_OUTPUTS 45
#define STEPPER_SET_MINIMUM_PULSE_WIDTH 46
#define STEPPER_SET_ENABLE_PIN 47
#define STEPPER_SET_3_PINS_INVERTED 48
#define STEPPER_SET_4_PINS_INVERTED 49
#define STEPPER_IS_RUNNING 50
#define STEPPER_GET_CURRENT_POSITION 51
#define STEPPER_GET_DISTANCE_TO_GO 52
#define STEPPER_GET_TARGET_POSITION 53
#define GET_FEATURES 54

// OASIS extensions to Telemetrix protocol
#define SET_MEMORY_REPORTING_INTERVAL 55
#define CPU_FAN_PWM_ATTACH 56
#define CPU_FAN_PWM_DETACH 57
#define CPU_FAN_TACH_ATTACH 58
#define CPU_FAN_TACH_DETACH 59
#define SET_CPU_FAN_SAMPLING_INTERVAL 60
#define CPU_FAN_WRITE 61
#define I2C_CCS811_BEGIN 62
#define I2C_CCS811_END 63
#define I2C_MPU6050_BEGIN 64
#define I2C_MPU6050_END 65
#define GET_UPTIME 66

// Maximum length of a command in bytes
#define MAX_COMMAND_LENGTH 30

namespace OASIS
{
class TelemetrixServer;

class TelemetrixCommands
{
public:
  static void RegisterServer(TelemetrixServer* server);

  // Retrieve the next command from the serial link
  static void GetNextCommand();

  // Returns true if a command is currently incoming on the serial link
  static bool IsInCommand() { return m_inCommand; }

  //////////////////////////////////////////////////////////////////////////////
  // Command functions
  //////////////////////////////////////////////////////////////////////////////

  // A function to loop back data over the serial port
  static void serial_loopback();

  // Set a pin to digital input, digital input_pullup, digital output,
  // and analog input. PWM is considered digital output, and i2c, spi, dht,
  // sonar, servo, and onewire have their own init methods.
  static void set_pin_mode();

  // set the state of digital output pin
  static void digital_write();

  // set the pwm value for a digital output pin
  // The term analog is confusing here, but it is what
  // Arduino uses.
  static void analog_write();

  // This method allows you modify what reports are generated.
  // You can disable all reports, including dhts, and sonar.
  // You can disable only digital and analog reports on a
  // pin basis, or enable those on a pin basis.
  static void modify_reporting();

  // Return the firmware version number
  static void get_firmware_version();

  // Query the firmware for the Arduino ID in use
  static void are_you_there();

  // Query the firmware for its uptime
  static void get_uptime();

  //////////////////////////////////////////////////////////////////////////////
  // Servo commands
  //////////////////////////////////////////////////////////////////////////////

  // Associate a pin with a servo
  static void servo_attach();

  // set a servo to a given angle
  static void servo_write();

  // detach a servo and make it available for future use
  static void servo_detach();

  //////////////////////////////////////////////////////////////////////////////
  // I2C functions
  //////////////////////////////////////////////////////////////////////////////

  // initialize i2c data transfers
  static void i2c_begin();

  // read a number of bytes from a specific i2c register
  static void i2c_read();

  // write a specified number of bytes to an i2c device
  static void i2c_write();

  //////////////////////////////////////////////////////////////////////////////
  // Sonar functions
  //////////////////////////////////////////////////////////////////////////////

  // associate 2 pins as trigger and echo pins for a sonar device
  static void sonar_new();

  //////////////////////////////////////////////////////////////////////////////
  // DHT functions
  //////////////////////////////////////////////////////////////////////////////

  static void dht_new();

  //////////////////////////////////////////////////////////////////////////////
  // Pin commands
  //////////////////////////////////////////////////////////////////////////////

  // stop all reports from being generated
  static void stop_all_reports();

  // set the analog scanning interval
  static void set_analog_scanning_interval();

  // enable all reports to be generated
  static void enable_all_reports();

  // reset the internal data structures to a known state
  static void reset_data();

  //////////////////////////////////////////////////////////////////////////////
  // SPI commands
  //////////////////////////////////////////////////////////////////////////////

  // initialize the SPI interface
  static void init_spi();

  // write a number of blocks to the SPI device
  static void write_blocking_spi();

  // read a number of bytes from the SPI device
  static void read_blocking_spi();

  // modify the SPI format
  static void set_format_spi();

  // set the SPI chip select line
  static void spi_cs_control();

  //////////////////////////////////////////////////////////////////////////////
  // One wire commands
  //////////////////////////////////////////////////////////////////////////////

  // Initialize the OneWire interface
  static void onewire_init();

  // send a OneWire reset
  static void onewire_reset();

  // send a OneWire select
  static void onewire_select();

  // send a OneWire skip
  static void onewire_skip();

  // write 1 byte to the OneWire device
  static void onewire_write();

  // read one byte from the OneWire device
  static void onewire_read();

  // Send a OneWire reset search command
  static void onewire_reset_search();

  // Send a OneWire search command
  static void onewire_search();

  // Calculate a OneWire CRC8 on a buffer containing a specified number of bytes
  static void onewire_crc8();

  //////////////////////////////////////////////////////////////////////////////
  // Stepper motor commands
  //////////////////////////////////////////////////////////////////////////////

  static void set_pin_mode_stepper();
  static void stepper_move_to();
  static void stepper_move();
  static void stepper_run();
  static void stepper_run_speed();
  static void stepper_set_max_speed();
  static void stepper_set_acceleration();
  static void stepper_set_speed();
  static void stepper_get_distance_to_go();
  static void stepper_get_target_position();
  static void stepper_get_current_position();
  static void stepper_set_current_position();
  static void stepper_run_speed_to_position();
  static void stepper_stop();
  static void stepper_disable_outputs();
  static void stepper_enable_outputs();
  static void stepper_set_minimum_pulse_width();
  static void stepper_set_enable_pin();
  static void stepper_set_3_pins_inverted();
  static void stepper_set_4_pins_inverted();
  static void stepper_is_running();

  //////////////////////////////////////////////////////////////////////////////
  // Feature support
  //////////////////////////////////////////////////////////////////////////////

  // Retrieve the features byte
  static void get_features();

  //////////////////////////////////////////////////////////////////////////////
  // Diagnostics
  //////////////////////////////////////////////////////////////////////////////

  static void set_memory_reporting_interval();

  //////////////////////////////////////////////////////////////////////////////
  // CPU fan commands
  //////////////////////////////////////////////////////////////////////////////

  // Associate a PWM pin for a 4-wire CPU fan
  static void cpu_fan_pwm_attach();

  // Disassociate a PWM pin for a 4-wire CPU fan
  static void cpu_fan_pwm_detach();

  // Associate a tachometer pin for a 4-wire CPU fan
  static void cpu_fan_tach_attach();

  // Disassociate a tachometer pin for a 4-wire CPU fan
  static void cpu_fan_tach_detach();

  // Set the tachometer sampling interval
  static void set_cpu_fan_sampling_interval();

  // Write a PWM value to a pin controlling a fan
  static void cpu_fan_write();

  //////////////////////////////////////////////////////////////////////////////
  // Air quality commands
  //////////////////////////////////////////////////////////////////////////////

  static void i2c_ccs811_begin();

  static void i2c_ccs811_end();

  //////////////////////////////////////////////////////////////////////////////
  // IMU commands
  //////////////////////////////////////////////////////////////////////////////

  static void i2c_mpu6050_begin();

  static void i2c_mpu6050_end();

private:
  // Registered server. Caller must call RegisterServer().
  static TelemetrixServer* m_server;

  // Buffer to hold incoming command data
  static uint8_t commandBuffer[MAX_COMMAND_LENGTH];

  // Critical section guard
  static volatile bool m_inCommand;
};
} // namespace OASIS
