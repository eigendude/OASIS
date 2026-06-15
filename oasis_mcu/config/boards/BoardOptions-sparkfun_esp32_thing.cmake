################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# Arduino boards.
# Platform: esp32
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

set(ARDUINO_BOARD "ESP32 Dev Module [esp32.esp32.esp32]") # ESP32 Dev Module

#==============================================================================
# Menu options.
# Board: ESP32 Dev Module [esp32.esp32.esp32]
#==============================================================================

# Option: CPU Frequency
set(ARDUINO_ESP32_ESP32_MENU_CPUFREQ_240 TRUE) # 240MHz (WiFi/BT)

# Option: Flash Frequency
set(ARDUINO_ESP32_ESP32_MENU_FLASHFREQ_80 TRUE) # 80MHz

# Option: Flash Mode
set(ARDUINO_ESP32_ESP32_MENU_FLASHMODE_DIO TRUE) # DIO

# Option: Flash Size
set(ARDUINO_ESP32_ESP32_MENU_FLASHSIZE_4M TRUE) # 4MB (32Mb)

# Option: Upload Speed
set(ARDUINO_ESP32_ESP32_MENU_UPLOADSPEED_921600 TRUE) # 921600
