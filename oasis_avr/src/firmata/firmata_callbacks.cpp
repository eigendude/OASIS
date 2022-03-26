/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from FirmataExpress under the AGPL 3.0 License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *  Copyright (C) 2018-2019 Alan Yorinks. All Rights Reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata_callbacks.hpp"

#include "firmata_extra.hpp"
#include "firmata_thread.hpp"
#include "firmata_utils.hpp"

#if defined(ENABLE_ANALOG)
#include "firmata_analog.hpp"
#endif

#if defined(ENABLE_DHT)
#include "firmata_dht.hpp"
#endif

#if defined(ENABLE_DIAGNOSTICS)
#include "firmata_diagnostics.hpp"
#endif

#if defined(ENABLE_DIGITAL)
#include "firmata_digital.hpp"
#endif

#if defined(ENABLE_I2C)
#include "firmata_i2c.hpp"

#include <Wire.h> // TODO: Move to I2C subsystem
#endif

#if defined(ENABLE_SERVO)
#include "firmata_servo.hpp"
#endif

#if defined(ENABLE_SONAR)
#include "firmata_sonar.hpp"
#endif

#if defined(ENABLE_SPI)
#include "firmata_spi.hpp"
#endif

#if defined(ENABLE_STEPPER)
#include "firmata_stepper.hpp"
#endif

#include <Arduino.h>
#include <Boards.h>
#include <FirmataExpress.h>

using namespace OASIS;

namespace OASIS
{

// Firmata constants
constexpr uint8_t ARDUINO_INSTANCE_ID = 1;

} // namespace OASIS

FirmataThread* FirmataCallbacks::m_thread = nullptr;

void FirmataCallbacks::InitializeCallbacks(FirmataThread& thread)
{
  // Initialize state
  m_thread = &thread;

  // Initialize Firmata
  Firmata.attach(ANALOG_MESSAGE, PWMWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, DigitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, ReportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, ReportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, SetPinModeCallback);
  Firmata.attach(SET_DIGITAL_PIN_VALUE, SetPinValueCallback);
  Firmata.attach(START_SYSEX, SysexCallback);
  Firmata.attach(SYSTEM_RESET, SystemResetCallback);
}

void FirmataCallbacks::PWMWriteCallback(uint8_t pin, int analogValue)
{
  if (pin < TOTAL_PINS)
  {
    switch (Firmata.getPinMode(pin))
    {
      case PIN_MODE_PWM:
      {
#if defined(ENABLE_DIGITAL)
        if (IS_PIN_PWM(pin))
          analogWrite(PIN_TO_PWM(pin), analogValue);

        Firmata.setPinState(pin, analogValue);
#else
        Firmata.sendString("Digital not enabled");
#endif
        break;
      }

      case PIN_MODE_SERVO:
      {
#if defined(ENABLE_SERVO)
        m_thread->GetServo()->WriteServo(pin, analogValue);

        Firmata.setPinState(pin, analogValue);
#else
        Firmata.sendString("Servo not enabled");
#endif

        break;
      }

      default:
        break;
    }
  }
}

void FirmataCallbacks::DigitalWriteCallback(uint8_t digitalPort, int portValue)
{
#if defined(ENABLE_DIGITAL)
  uint8_t mask = 1;
  uint8_t pinWriteMask = 0;

  if (digitalPort < TOTAL_PORTS)
  {
    // Create a mask of the pins on this port that are writable.
    uint8_t lastPin = digitalPort * 8 + 8;

    if (lastPin > TOTAL_PINS)
      lastPin = TOTAL_PINS;

    for (uint8_t pin = digitalPort * 8; pin < lastPin; ++pin)
    {
      // Do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin))
      {
        // Do not touch pins in PWM, ANALOG, SERVO or other modes
        if (Firmata.getPinMode(pin) == PIN_MODE_OUTPUT || Firmata.getPinMode(pin) == PIN_MODE_INPUT)
        {
          uint8_t pinValue = (static_cast<uint8_t>(portValue) & mask) ? 1 : 0;

          if (Firmata.getPinMode(pin) == PIN_MODE_OUTPUT)
          {
            pinWriteMask |= mask;
          }
          else if (Firmata.getPinMode(pin) == PIN_MODE_INPUT && pinValue == 1 &&
                   Firmata.getPinState(pin) != 1)
          {
            // Only handle INPUT here for backwards compatibility
            pinMode(pin, INPUT_PULLUP);
          }

          Firmata.setPinState(pin, pinValue);
        }
      }

      mask = mask << 1;
    }

    writePort(digitalPort, static_cast<uint8_t>(portValue), pinWriteMask);
  }
#else
  Firmata.sendString("Digital not enabled");
#endif
}

void FirmataCallbacks::ReportAnalogCallback(uint8_t analogPin, int enableReporting)
{
#if defined(ENABLE_ANALOG)
  if (analogPin < TOTAL_ANALOG_PINS)
  {
    m_thread->GetAnalog()->EnableAnalogInput(analogPin, enableReporting != 0);

    if (enableReporting != 0)
    {
      // Prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!m_thread->IsResetting())
      {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, WiFi or Bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
      }
    }
  }

  // TODO: Save status to EEPROM here, if changed
#else
  Firmata.sendString("Analog not enabled");
#endif
}

void FirmataCallbacks::ReportDigitalCallback(uint8_t digitalPort, int enableReporting)
{
#if defined(ENABLE_DIGITAL)
  if (digitalPort < TOTAL_PORTS)
  {
    m_thread->GetDigital()->EnableDigitalInput(digitalPort, enableReporting != 0);

    // Send port value immediately. This is helpful when connected via ethernet,
    // WiFi or Bluetooth so pin states can be known upon reconnecting.
    if (enableReporting != 0)
      m_thread->GetDigital()->SendPort(digitalPort);
  }

  // Do not disable analog reporting on these 8 pins, to allow some pins to be
  // used for digital, others analog.
  //
  // Instead, allow both types of reporting to be enabled, but check if the
  // pin is configured as analog when sampling the analog inputs.
  //
  // Likewise, while scanning digital pins, portConfigInputs will mask off
  // values from any pins configured as analog.
#else
  Firmata.sendString("Digital not enabled");
#endif
}

void FirmataCallbacks::SetPinModeCallback(uint8_t pin, int mode)
{
  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;

#if defined(ENABLE_ANALOG)
  if (IS_PIN_ANALOG(pin))
  {
    // Turn on/off reporting
    ReportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0);
  }
#endif

#if defined(ENABLE_DIGITAL)
  if (IS_PIN_DIGITAL(pin))
  {
    m_thread->GetDigital()->SetDigitalPinMode(pin, mode);
  }
#endif

#if defined(ENABLE_I2C)
  // Prepare for new pin mode
  if (Firmata.getPinMode(pin) == PIN_MODE_I2C && m_thread->GetI2C()->IsI2CEnabled() &&
      mode != PIN_MODE_I2C)
  {
    // Disable I2C so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    m_thread->GetI2C()->DisableI2CPins();
  }
#endif

#if defined(ENABLE_SERVO)
  if (IS_PIN_DIGITAL(pin) && mode != PIN_MODE_SERVO)
  {
    m_thread->GetServo()->PrepareServoPin(pin);
  }
#endif

#if defined(ENABLE_SPI)
  if (IS_PIN_SPI(pin))
  {
    if (mode != PIN_MODE_SPI && m_thread->GetSPI()->IsSpiEnabled())
    {
      // Disable SPI so pins can be used for other functions. The following if
      // statements should reconfigure the pins properly
      if (Firmata.getPinMode(pin) == PIN_MODE_SPI)
        m_thread->GetSPI()->DisableSpiPins();
    }
  }
#endif

  // Update Firmata state
  Firmata.setPinState(pin, 0);

  // Handle pin mode change
  switch (mode)
  {
    case PIN_MODE_ANALOG:
    {
#if defined(ENABLE_ANALOG)
      if (IS_PIN_ANALOG(pin))
      {
        if (IS_PIN_DIGITAL(pin))
        {
          // Disable output driver
          pinMode(PIN_TO_DIGITAL(pin), INPUT);
        }

        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
#else
      Firmata.sendString("Analog not enabled");
#endif
      break;
    }

    case PIN_MODE_DHT:
    {
#if defined(ENABLE_DHT)
      Firmata.setPinMode(pin, PIN_MODE_DHT);
#else
      Firmata.sendString("DHT not enabled");
#endif
      break;
    }

    case PIN_MODE_INPUT:
    {
#if defined(ENABLE_DIGITAL)
      if (IS_PIN_DIGITAL(pin))
      {
        // Disable output driver
        pinMode(PIN_TO_DIGITAL(pin), INPUT);

        Firmata.setPinMode(pin, PIN_MODE_INPUT);
        Firmata.setPinState(pin, 0);
      }
#else
      Firmata.sendString("Digital not enabled");
#endif
      break;
    }

    case PIN_MODE_PULLUP:
    {
#if defined(ENABLE_DIGITAL)
      if (IS_PIN_DIGITAL(pin))
      {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);

        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
#else
      Firmata.sendString("Digital not enabled");
#endif
      break;
    }

    case PIN_MODE_OUTPUT:
    {
#if defined(ENABLE_DIGITAL)
      if (IS_PIN_DIGITAL(pin))
      {
        if (Firmata.getPinMode(pin) == PIN_MODE_PWM)
        {
          // Disable PWM if pin mode was previously set to PWM.
          digitalWrite(PIN_TO_DIGITAL(pin), LOW);
        }

        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);

        Firmata.setPinMode(pin, PIN_MODE_OUTPUT);
      }
#else
      Firmata.sendString("Digital not enabled");
#endif
      break;
    }

    case PIN_MODE_PWM:
    {
#if defined(ENABLE_DIGITAL)
      if (IS_PIN_PWM(pin))
      {
        pinMode(PIN_TO_PWM(pin), OUTPUT);

        analogWrite(PIN_TO_PWM(pin), 0);

        Firmata.setPinMode(pin, PIN_MODE_PWM);
      }
#else
      Firmata.sendString("Digital not enabled");
#endif
      break;
    }

    case PIN_MODE_I2C:
    {
#if defined(ENABLE_I2C)
      if (IS_PIN_I2C(pin))
      {
        // Mark the pin as I2C. The user must call I2C_CONFIG to enable I2C for
        // a device
        Firmata.setPinMode(pin, PIN_MODE_I2C);
      }
#else
      Firmata.sendString("I2C not enabled");
#endif
      break;
    }

    case PIN_MODE_SERVO:
    {
#if defined(ENABLE_SERVO)
      if (IS_PIN_DIGITAL(pin))
      {
        Firmata.setPinMode(pin, PIN_MODE_SERVO);

        if (m_thread->GetServo() != nullptr)
          m_thread->GetServo()->SetServoPin(pin);
      }
#else
      Firmata.sendString("Servo not enabled");
#endif
      break;
    }

    case PIN_MODE_SONAR:
    {
#if defined(ENABLE_SONAR)
      Firmata.setPinMode(pin, PIN_MODE_SONAR);
#else
      Firmata.sendString("Sonar not enabled");
#endif
      break;
    }

    case PIN_MODE_SPI:
    {
#if defined(ENABLE_SPI)
      // SPI is enabled when the SPI_DATA sysex command is sent
#else
      Firmata.sendString("SPI not enabled");
#endif
      break;
    }

    case PIN_MODE_STEPPER:
    {
#if defined(ENABLE_STEPPER)
      Firmata.setPinMode(pin, PIN_MODE_STEPPER);
#else
      Firmata.sendString("Stepper not enabled");
#endif
      break;
    }

    default:
    {
      // TODO: Put error msgs in EEPROM
      Firmata.sendString("Unknown pin mode");
      break;
    }
  }

  // TODO: Save status to EEPROM here, if changed
}

void FirmataCallbacks::SetPinValueCallback(uint8_t pin, int value)
{
#if defined(ENABLE_DIGITAL)
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin))
  {
    if (Firmata.getPinMode(pin) == PIN_MODE_OUTPUT)
    {
      Firmata.setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
#else
  Firmata.sendString("Digital not enabled");
#endif
}

void FirmataCallbacks::SysexCallback(uint8_t command, uint8_t argc, uint8_t* argv)
{
  // Local state
  uint8_t mode = 0;
  uint8_t stopTX = 0;
  uint8_t slaveAddress = 0;
  uint8_t data = 0;
  int slaveRegister = 0;
  unsigned int delayTime = 0;
  uint8_t pin = 0;
  int frequency = 0;
  int duration = 0;

  switch (command)
  {
    case RU_THERE:
    {
      Firmata.write(START_SYSEX);
      Firmata.write(static_cast<uint8_t>(I_AM_HERE));
      Firmata.write(static_cast<uint8_t>(ARDUINO_INSTANCE_ID));
      Firmata.write(END_SYSEX);

      break;
    }

    case SAMPLING_INTERVAL:
    {
      if (argc > 1)
      {
        const uint8_t samplingIntervalMs = argv[0] + (argv[1] << 7);
        m_thread->SetSamplingInterval(samplingIntervalMs);
      }
      else
      {
        // This was commented in FirmataExpress.ino
        //Firmata.sendString("Not enough data");
      }

      break;
    }

    case EXTENDED_ANALOG:
    {
      if (argc > 1)
      {
        int val = argv[1];

        if (argc > 2)
          val |= (argv[2] << 7);

        if (argc > 3)
          val |= (argv[3] << 14);

        PWMWriteCallback(argv[0], val);
      }

      break;
    }

    case CAPABILITY_QUERY:
    {
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (uint8_t pin = 0; pin < TOTAL_PINS; ++pin)
      {
#if defined(ENABLE_DIGITAL)
        if (IS_PIN_DIGITAL(pin))
        {
          Firmata.write(static_cast<uint8_t>(PIN_MODE_INPUT));
          Firmata.write(1);
          Firmata.write(static_cast<uint8_t>(PIN_MODE_PULLUP));
          Firmata.write(1);
          Firmata.write(static_cast<uint8_t>(PIN_MODE_OUTPUT));
          Firmata.write(1);
          Firmata.write(static_cast<uint8_t>(PIN_MODE_STEPPER));
          Firmata.write(1);
          Firmata.write(static_cast<uint8_t>(PIN_MODE_SONAR));
          Firmata.write(1);
          Firmata.write(static_cast<uint8_t>(PIN_MODE_DHT));
          Firmata.write(1);
        }

        if (IS_PIN_PWM(pin))
        {
          Firmata.write(PIN_MODE_PWM);
          Firmata.write(DEFAULT_PWM_RESOLUTION);
        }
#endif

#if defined(ENABLE_ANALOG)
        if (IS_PIN_ANALOG(pin))
        {
          Firmata.write(PIN_MODE_ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }
#endif

#if defined(ENABLE_SPI)
        if (IS_PIN_SPI(pin))
        {
          Firmata.write(PIN_MODE_SPI);
          Firmata.write(1); // TODO: Could assign a number to map SPI pins
        }
#endif

        Firmata.write(127);
      }

      Firmata.write(END_SYSEX);

      break;
    }

    case PIN_STATE_QUERY:
    {
      if (argc > 0)
      {
        uint8_t pin = argv[0];

        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);

        if (pin < TOTAL_PINS)
        {
          Firmata.write(Firmata.getPinMode(pin));
          Firmata.write(static_cast<uint8_t>(Firmata.getPinState(pin)) & 0x7F);

          if (Firmata.getPinState(pin) & 0xFF80)
            Firmata.write(static_cast<uint8_t>(Firmata.getPinState(pin) >> 7) & 0x7F);

          if (Firmata.getPinState(pin) & 0xC000)
            Firmata.write(static_cast<uint8_t>(Firmata.getPinState(pin) >> 14) & 0x7F);
        }

        Firmata.write(END_SYSEX);
      }

      break;
    }

    case ANALOG_MAPPING_QUERY:
    {
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);

      for (uint8_t pin = 0; pin < TOTAL_PINS; ++pin)
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);

      Firmata.write(END_SYSEX);

      break;
    }

    case FIRMATA_MEMORY_CONFIG:
    {
#if defined(ENABLE_DIAGNOSTICS)
      if (argc > 0)
      {
        uint32_t memoryPeriodMs = argv[0];

        if (argc > 1)
          memoryPeriodMs |= (argv[1] << 7);

        if (argc > 2)
          memoryPeriodMs |= (argv[2] << 14);

        if (argc > 3)
          memoryPeriodMs |= (argv[3] << 21);

        m_thread->GetDiagnostics()->ConfigureMemoryReporting(memoryPeriodMs);
      }
#else
      Firmata.sendString("Diagnostics not enabled");
#endif
      break;
    }

    case DHT_CONFIG:
    {
#if defined(ENABLE_DHT)
      const int DHT_pin = argv[0];
      const int DHT_type = argv[1];

      m_thread->GetDHT()->ConfigureDHT(DHT_pin, DHT_type);
#else
      Firmata.sendString("DHT not enabled");
#endif

      break;
    }

    case I2C_REQUEST:
    {
#if defined(ENABLE_I2C)
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK)
      {
        Firmata.sendString("10-bit addressing not supported");
        return;
      }
      else
      {
        slaveAddress = argv[0];
      }

      // Need to invert the logic here since 0 will be default for client
      // libraries that have not updated to add support for restart tx
      if (argv[1] & I2C_END_TX_MASK)
      {
        stopTX = I2C_RESTART_TX;
      }
      else
      {
        stopTX = I2C_STOP_TX; // Default
      }

      switch (mode)
      {
        case I2C_WRITE:
        {
          //m_thread->I2CWrite()

          Wire.beginTransmission(slaveAddress);

          for (uint8_t i = 2; i < argc; i += 2)
          {
            data = argv[i] + (argv[i + 1] << 7);
            Wire.write(data);
          }

          Wire.endTransmission();

          delayMicroseconds(70);

          break;
        }

        case I2C_READ:
        {
          if (m_thread->GetI2C() == nullptr)
            break;

          if (argc == 6)
          {
            // A slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);

            // Bytes to read
            data = argv[4] + (argv[5] << 7);
          }
          else
          {
            // A slave register is NOT specified
            slaveRegister = I2C_REGISTER_NOT_SPECIFIED;

            // Bytes to read
            data = argv[2] + (argv[3] << 7);
          }

          m_thread->GetI2C()->ReadAndReportI2CData(slaveAddress, static_cast<int>(slaveRegister),
                                                   data, stopTX);

          break;
        }

        case I2C_READ_CONTINUOUSLY:
        {
          if (m_thread->GetI2C() == nullptr)
            break;

          if (m_thread->GetI2C()->GetI2CQueryCount() >= I2C_MAX_QUERIES)
          {
            // Too many queries, just ignore
            Firmata.sendString("too many queries");

            break;
          }

          if (argc == 6)
          {
            // A slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7); // bytes to read
          }
          else
          {
            // A slave register is NOT specified
            slaveRegister = static_cast<int>(I2C_REGISTER_NOT_SPECIFIED);
            data = argv[2] + (argv[3] << 7); // bytes to read
          }

          m_thread->GetI2C()->AddI2CQuery(slaveAddress, slaveRegister, data, stopTX);

          break;
        }

        case I2C_STOP_READING:
        {
          if (m_thread->GetI2C() == nullptr)
            break;

          uint8_t queryIndexToSkip = 0;

          // If read continuous mode is enabled for only a single I2C device,
          // disable read continuous reporting for that device
          if (m_thread->GetI2C()->GetI2CQueryCount() == 1)
          {
            m_thread->GetI2C()->DisableI2CReporting();
          }
          else
          {
            queryIndexToSkip = 0;

            // If read continuous mode is enabled for multiple devices,
            // determine which device to stop reading and remove its data from
            // the array, shifting other array data to fill the space
            for (uint8_t i = 0; i < m_thread->GetI2C()->GetI2CQueryCount(); ++i)
            {
              if (m_thread->GetI2C()->GetI2CQueryAddress(i) == slaveAddress)
              {
                queryIndexToSkip = i;
                break;
              }
            }

            for (uint8_t i = queryIndexToSkip; i < m_thread->GetI2C()->GetI2CQueryCount(); ++i)
            {
              if (i < I2C_MAX_QUERIES)
                m_thread->GetI2C()->RemoveI2CQuery(i);
            }

            m_thread->GetI2C()->SetPreviousI2CQuery();
          }

          break;
        }

        default:
          break;
      }
#else
      Firmata.sendString("I2C not enabled");
#endif

      break;
    }

    case I2C_CONFIG:
    {
#if defined(ENABLE_I2C)
      delayTime = (argv[0] + (argv[1] << 7));

      if (argc > 1 && delayTime > 0)
        m_thread->GetI2C()->SetI2CReadDelayTime(delayTime);

      if (!m_thread->GetI2C()->IsI2CEnabled())
        m_thread->GetI2C()->EnableI2CPins();
#else
      Firmata.sendString("I2C not enabled");
#endif

      break;
    }

    case SERVO_CONFIG:
    {
#if defined(ENABLE_SERVO)
      if (argc > 4)
      {
        // These vars are here for clarity, they'll optimized away by the compiler
        uint8_t pin = argv[0];
        int minPulse = argv[1] + (argv[2] << 7);
        int maxPulse = argv[3] + (argv[4] << 7);

        if (IS_PIN_DIGITAL(pin))
        {
          if (m_thread->GetServo()->GetServoPin(pin) < MAX_SERVOS &&
              m_thread->GetServo()->IsServoAttached(pin))
            m_thread->GetServo()->DetachServo(pin);

          m_thread->GetServo()->AttachServo(pin, minPulse, maxPulse);

          SetPinModeCallback(pin, PIN_MODE_SERVO);
        }
      }
#else
      Firmata.sendString("Servo not enabled");
#endif

      break;
    }

    case SONAR_CONFIG:
    {
#if defined(ENABLE_SONAR)
      // arg0 = trigger pin
      // arg1 = echo pin
      // arg2 = timeout_lsb
      // arg3 = timeout_msb

      unsigned long timeout = 0;
      if (m_thread->GetSonar()->GetActiveSonarCount() < MAX_SONARS)
      {
        const uint8_t sonarTriggerPin = argv[0];
        const uint8_t sonarEchoPin = argv[1];
        const unsigned long timeout = argv[2] + (argv[3] << 7);

        m_thread->GetSonar()->AddSonar(sonarTriggerPin, sonarEchoPin, timeout);
      }
      else
      {
        Firmata.sendString("PING_CONFIG Error: Exceeded number of supported ping devices");
      }
#else
      Firmata.sendString("Sonar not enabled");
#endif

      break;
    }

    case SPI_DATA:
    {
#if defined(ENABLE_SPI)
      if (argc > 0)
      {
        const uint8_t command = argv[0];
        m_thread->GetSPI()->HandleSpiRequest(command, argc - 1, argv + 1);
      }
      else
      {
        Firmata.sendString("Error in SPI_DATA command: empty message");
      }
#endif

      break;
    }

    case STEPPER_DATA:
    {
#if defined(ENABLE_STEPPER)
      // Determine if this a STEPPER_CONFIGURE command or STEPPER_OPERATE command
      const uint8_t stepperCommand = argv[0];
      if (stepperCommand == STEPPER_COMMAND_CONFIGURE)
      {
        const int numSteps = argv[1] + (argv[2] << 7);
        const int pin1 = argv[3];
        const int pin2 = argv[4];

        if (argc == 5) // Two pin motor
        {
          m_thread->GetStepper()->CreateStepper(numSteps, pin1, pin2);
        }
        else if (argc == 7) // 4 wire motor
        {
          const int pin3 = argv[5];
          const int pin4 = argv[6];

          m_thread->GetStepper()->CreateStepper(numSteps, pin1, pin2, pin3, pin4);
        }
        else
        {
          Firmata.sendString("STEPPER CONFIG Error: Wrong Number of arguments");
          FirmataUtils::PrintData("argc = ", argc);
        }
      }
      else if (stepperCommand == STEPPER_COMMAND_STEP)
      {
        const long speed = static_cast<long>(argv[1]) | (static_cast<long>(argv[2]) << 7) |
                           (static_cast<long>(argv[3]) << 14);
        const int numSteps = argv[4] + (argv[5] << 7);
        const int direction = argv[6];

        m_thread->GetStepper()->StepMotor(speed, numSteps, direction);
      }
      else if (stepperCommand == STEPPER_COMMAND_LIBRARY_VERSION)
      {
        m_thread->GetStepper()->SendStepperLibraryVersion();
        break;
      }
      else
      {
        Firmata.sendString("STEPPER CONFIG Error: UNKNOWN STEPPER COMMAND");
      }
#else
      Firmata.sendString("Stepper not enabled");
#endif

      break;
    }

    default:
    {
      Firmata.sendString("Unknown sysex callback");
      break;
    }
  }
}

void FirmataCallbacks::SystemResetCallback()
{
  m_thread->Reset();
}
