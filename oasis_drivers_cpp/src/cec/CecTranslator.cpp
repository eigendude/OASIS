/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CecTranslator.h"

#include "utils/StringUtils.h"

#include <sstream>

using namespace OASIS;
using namespace OASIS::CEC;

namespace
{
// TODO: Replace with ::CEC::CEC_VENDOR_VIZIO2 when ID has been upstreamed
const uint32_t CEC_VENDOR_VIZIO2 = 0x9D1900;
} // namespace

std::string CecTranslator::TranslateAlert(::CEC::libcec_alert alert)
{
  switch (alert)
  {
    case ::CEC::CEC_ALERT_SERVICE_DEVICE:
      return "CEC_ALERT_SERVICE_DEVICE";
    case ::CEC::CEC_ALERT_CONNECTION_LOST:
      return "CEC_ALERT_CONNECTION_LOST";
    case ::CEC::CEC_ALERT_PERMISSION_ERROR:
      return "CEC_ALERT_PERMISSION_ERROR";
    case ::CEC::CEC_ALERT_PORT_BUSY:
      return "CEC_ALERT_PORT_BUSY";
    case ::CEC::CEC_ALERT_PHYSICAL_ADDRESS_ERROR:
      return "CEC_ALERT_PHYSICAL_ADDRESS_ERROR";
    case ::CEC::CEC_ALERT_TV_POLL_FAILED:
      return "CEC_ALERT_TV_POLL_FAILED";
    default:
      break;
  }

  return "unknown (" + std::to_string(static_cast<int>(alert)) + ")";
}

std::string CecTranslator::TranslateLogicalAddress(::CEC::cec_logical_address address)
{
  switch (address)
  {
    case ::CEC::CECDEVICE_UNKNOWN:
      return "CECDEVICE_UNKNOWN";
    case ::CEC::CECDEVICE_TV:
      return "CECDEVICE_TV";
    case ::CEC::CECDEVICE_RECORDINGDEVICE1:
      return "CECDEVICE_RECORDINGDEVICE1";
    case ::CEC::CECDEVICE_RECORDINGDEVICE2:
      return "CECDEVICE_RECORDINGDEVICE2";
    case ::CEC::CECDEVICE_TUNER1:
      return "CECDEVICE_TUNER1";
    case ::CEC::CECDEVICE_PLAYBACKDEVICE1:
      return "CECDEVICE_PLAYBACKDEVICE1";
    case ::CEC::CECDEVICE_AUDIOSYSTEM:
      return "CECDEVICE_AUDIOSYSTEM";
    case ::CEC::CECDEVICE_TUNER2:
      return "CECDEVICE_TUNER2";
    case ::CEC::CECDEVICE_TUNER3:
      return "CECDEVICE_TUNER3";
    case ::CEC::CECDEVICE_PLAYBACKDEVICE2:
      return "CECDEVICE_PLAYBACKDEVICE2";
    case ::CEC::CECDEVICE_RECORDINGDEVICE3:
      return "CECDEVICE_RECORDINGDEVICE3";
    case ::CEC::CECDEVICE_TUNER4:
      return "CECDEVICE_TUNER4";
    case ::CEC::CECDEVICE_PLAYBACKDEVICE3:
      return "CECDEVICE_PLAYBACKDEVICE3";
    case ::CEC::CECDEVICE_RESERVED1:
      return "CECDEVICE_RESERVED1";
    case ::CEC::CECDEVICE_RESERVED2:
      return "CECDEVICE_RESERVED2";
    case ::CEC::CECDEVICE_FREEUSE:
      return "CECDEVICE_FREEUSE";
    /* TODO: Duplicate value in libCEC?
  case ::CEC::CECDEVICE_UNREGISTERED:
    return "CECDEVICE_UNREGISTERED";
  */
    case ::CEC::CECDEVICE_BROADCAST:
      return "CECDEVICE_BROADCAST";
    default:
      break;
  }

  return "unknown (" + std::to_string(static_cast<int>(address)) + ")";
}

std::string CecTranslator::TranslateLogicalAddresses(const ::CEC::cec_logical_addresses& addresses)
{
  std::ostringstream devices;
  for (unsigned int iPtr = static_cast<unsigned int>(::CEC::CECDEVICE_TV);
       iPtr <= static_cast<unsigned int>(::CEC::CECDEVICE_BROADCAST); ++iPtr)
  {
    if (addresses[iPtr])
    {
      devices << CecTranslator::TranslateLogicalAddress(
          static_cast<::CEC::cec_logical_address>(iPtr));
      devices << ", ";
    }
  }

  std::string strDevices = devices.str();
  if (!strDevices.empty())
    strDevices.erase(strDevices.end() - 2, strDevices.end());

  return strDevices;
}

std::string CecTranslator::TranslateOpcode(::CEC::cec_opcode opcode)
{
  switch (opcode)
  {
    case ::CEC::CEC_OPCODE_ACTIVE_SOURCE:
      return "CEC_OPCODE_ACTIVE_SOURCE";
    case ::CEC::CEC_OPCODE_IMAGE_VIEW_ON:
      return "CEC_OPCODE_IMAGE_VIEW_ON";
    case ::CEC::CEC_OPCODE_TEXT_VIEW_ON:
      return "CEC_OPCODE_TEXT_VIEW_ON";
    case ::CEC::CEC_OPCODE_INACTIVE_SOURCE:
      return "CEC_OPCODE_INACTIVE_SOURCE";
    case ::CEC::CEC_OPCODE_REQUEST_ACTIVE_SOURCE:
      return "CEC_OPCODE_REQUEST_ACTIVE_SOURCE";
    case ::CEC::CEC_OPCODE_ROUTING_CHANGE:
      return "CEC_OPCODE_ROUTING_CHANGE";
    case ::CEC::CEC_OPCODE_ROUTING_INFORMATION:
      return "CEC_OPCODE_ROUTING_INFORMATION";
    case ::CEC::CEC_OPCODE_SET_STREAM_PATH:
      return "CEC_OPCODE_SET_STREAM_PATH";
    case ::CEC::CEC_OPCODE_STANDBY:
      return "CEC_OPCODE_STANDBY";
    case ::CEC::CEC_OPCODE_RECORD_OFF:
      return "CEC_OPCODE_RECORD_OFF";
    case ::CEC::CEC_OPCODE_RECORD_ON:
      return "CEC_OPCODE_RECORD_ON";
    case ::CEC::CEC_OPCODE_RECORD_STATUS:
      return "CEC_OPCODE_RECORD_STATUS";
    case ::CEC::CEC_OPCODE_RECORD_TV_SCREEN:
      return "CEC_OPCODE_RECORD_TV_SCREEN";
    case ::CEC::CEC_OPCODE_CLEAR_ANALOGUE_TIMER:
      return "CEC_OPCODE_CLEAR_ANALOGUE_TIMER";
    case ::CEC::CEC_OPCODE_CLEAR_DIGITAL_TIMER:
      return "CEC_OPCODE_CLEAR_DIGITAL_TIMER";
    case ::CEC::CEC_OPCODE_CLEAR_EXTERNAL_TIMER:
      return "CEC_OPCODE_CLEAR_EXTERNAL_TIMER";
    case ::CEC::CEC_OPCODE_SET_ANALOGUE_TIMER:
      return "CEC_OPCODE_SET_ANALOGUE_TIMER";
    case ::CEC::CEC_OPCODE_SET_DIGITAL_TIMER:
      return "CEC_OPCODE_SET_DIGITAL_TIMER";
    case ::CEC::CEC_OPCODE_SET_EXTERNAL_TIMER:
      return "CEC_OPCODE_SET_EXTERNAL_TIMER";
    case ::CEC::CEC_OPCODE_SET_TIMER_PROGRAM_TITLE:
      return "CEC_OPCODE_SET_TIMER_PROGRAM_TITLE";
    case ::CEC::CEC_OPCODE_TIMER_CLEARED_STATUS:
      return "CEC_OPCODE_TIMER_CLEARED_STATUS";
    case ::CEC::CEC_OPCODE_TIMER_STATUS:
      return "CEC_OPCODE_TIMER_STATUS";
    case ::CEC::CEC_OPCODE_CEC_VERSION:
      return "CEC_OPCODE_CEC_VERSION";
    case ::CEC::CEC_OPCODE_GET_CEC_VERSION:
      return "CEC_OPCODE_GET_CEC_VERSION";
    case ::CEC::CEC_OPCODE_GIVE_PHYSICAL_ADDRESS:
      return "CEC_OPCODE_GIVE_PHYSICAL_ADDRESS";
    case ::CEC::CEC_OPCODE_GET_MENU_LANGUAGE:
      return "CEC_OPCODE_GET_MENU_LANGUAGE";
    case ::CEC::CEC_OPCODE_REPORT_PHYSICAL_ADDRESS:
      return "CEC_OPCODE_REPORT_PHYSICAL_ADDRESS";
    case ::CEC::CEC_OPCODE_SET_MENU_LANGUAGE:
      return "CEC_OPCODE_SET_MENU_LANGUAGE";
    case ::CEC::CEC_OPCODE_DECK_CONTROL:
      return "CEC_OPCODE_DECK_CONTROL";
    case ::CEC::CEC_OPCODE_DECK_STATUS:
      return "CEC_OPCODE_DECK_STATUS";
    case ::CEC::CEC_OPCODE_GIVE_DECK_STATUS:
      return "CEC_OPCODE_GIVE_DECK_STATUS";
    case ::CEC::CEC_OPCODE_PLAY:
      return "CEC_OPCODE_PLAY";
    case ::CEC::CEC_OPCODE_GIVE_TUNER_DEVICE_STATUS:
      return "CEC_OPCODE_GIVE_TUNER_DEVICE_STATUS";
    case ::CEC::CEC_OPCODE_SELECT_ANALOGUE_SERVICE:
      return "CEC_OPCODE_SELECT_ANALOGUE_SERVICE";
    case ::CEC::CEC_OPCODE_SELECT_DIGITAL_SERVICE:
      return "CEC_OPCODE_SELECT_DIGITAL_SERVICE";
    case ::CEC::CEC_OPCODE_TUNER_DEVICE_STATUS:
      return "CEC_OPCODE_TUNER_DEVICE_STATUS";
    case ::CEC::CEC_OPCODE_TUNER_STEP_DECREMENT:
      return "CEC_OPCODE_TUNER_STEP_DECREMENT";
    case ::CEC::CEC_OPCODE_TUNER_STEP_INCREMENT:
      return "CEC_OPCODE_TUNER_STEP_INCREMENT";
    case ::CEC::CEC_OPCODE_DEVICE_VENDOR_ID:
      return "CEC_OPCODE_DEVICE_VENDOR_ID";
    case ::CEC::CEC_OPCODE_GIVE_DEVICE_VENDOR_ID:
      return "CEC_OPCODE_GIVE_DEVICE_VENDOR_ID";
    case ::CEC::CEC_OPCODE_VENDOR_COMMAND:
      return "CEC_OPCODE_VENDOR_COMMAND";
    case ::CEC::CEC_OPCODE_VENDOR_COMMAND_WITH_ID:
      return "CEC_OPCODE_VENDOR_COMMAND_WITH_ID";
    case ::CEC::CEC_OPCODE_VENDOR_REMOTE_BUTTON_DOWN:
      return "CEC_OPCODE_VENDOR_REMOTE_BUTTON_DOWN";
    case ::CEC::CEC_OPCODE_VENDOR_REMOTE_BUTTON_UP:
      return "CEC_OPCODE_VENDOR_REMOTE_BUTTON_UP";
    case ::CEC::CEC_OPCODE_SET_OSD_STRING:
      return "CEC_OPCODE_SET_OSD_STRING";
    case ::CEC::CEC_OPCODE_GIVE_OSD_NAME:
      return "CEC_OPCODE_GIVE_OSD_NAME";
    case ::CEC::CEC_OPCODE_SET_OSD_NAME:
      return "CEC_OPCODE_SET_OSD_NAME";
    case ::CEC::CEC_OPCODE_MENU_REQUEST:
      return "CEC_OPCODE_MENU_REQUEST";
    case ::CEC::CEC_OPCODE_MENU_STATUS:
      return "CEC_OPCODE_MENU_STATUS";
    case ::CEC::CEC_OPCODE_USER_CONTROL_PRESSED:
      return "CEC_OPCODE_USER_CONTROL_PRESSED";
    case ::CEC::CEC_OPCODE_USER_CONTROL_RELEASE:
      return "CEC_OPCODE_USER_CONTROL_RELEASE";
    case ::CEC::CEC_OPCODE_GIVE_DEVICE_POWER_STATUS:
      return "CEC_OPCODE_GIVE_DEVICE_POWER_STATUS";
    case ::CEC::CEC_OPCODE_REPORT_POWER_STATUS:
      return "CEC_OPCODE_REPORT_POWER_STATUS";
    case ::CEC::CEC_OPCODE_FEATURE_ABORT:
      return "CEC_OPCODE_FEATURE_ABORT";
    case ::CEC::CEC_OPCODE_ABORT:
      return "CEC_OPCODE_ABORT";
    case ::CEC::CEC_OPCODE_GIVE_AUDIO_STATUS:
      return "CEC_OPCODE_GIVE_AUDIO_STATUS";
    case ::CEC::CEC_OPCODE_GIVE_SYSTEM_AUDIO_MODE_STATUS:
      return "CEC_OPCODE_GIVE_SYSTEM_AUDIO_MODE_STATUS";
    case ::CEC::CEC_OPCODE_REPORT_AUDIO_STATUS:
      return "CEC_OPCODE_REPORT_AUDIO_STATUS";
    case ::CEC::CEC_OPCODE_SET_SYSTEM_AUDIO_MODE:
      return "CEC_OPCODE_SET_SYSTEM_AUDIO_MODE";
    case ::CEC::CEC_OPCODE_SYSTEM_AUDIO_MODE_REQUEST:
      return "CEC_OPCODE_SYSTEM_AUDIO_MODE_REQUEST";
    case ::CEC::CEC_OPCODE_SYSTEM_AUDIO_MODE_STATUS:
      return "CEC_OPCODE_SYSTEM_AUDIO_MODE_STATUS";
    case ::CEC::CEC_OPCODE_SET_AUDIO_RATE:
      return "CEC_OPCODE_SET_AUDIO_RATE";
    case ::CEC::CEC_OPCODE_REPORT_SHORT_AUDIO_DESCRIPTORS:
      return "CEC_OPCODE_REPORT_SHORT_AUDIO_DESCRIPTORS";
    case ::CEC::CEC_OPCODE_REQUEST_SHORT_AUDIO_DESCRIPTORS:
      return "CEC_OPCODE_REQUEST_SHORT_AUDIO_DESCRIPTORS";
    case ::CEC::CEC_OPCODE_START_ARC:
      return "CEC_OPCODE_START_ARC";
    case ::CEC::CEC_OPCODE_REPORT_ARC_STARTED:
      return "CEC_OPCODE_REPORT_ARC_STARTED";
    case ::CEC::CEC_OPCODE_REPORT_ARC_ENDED:
      return "CEC_OPCODE_REPORT_ARC_ENDED";
    case ::CEC::CEC_OPCODE_REQUEST_ARC_START:
      return "CEC_OPCODE_REQUEST_ARC_START";
    case ::CEC::CEC_OPCODE_REQUEST_ARC_END:
      return "CEC_OPCODE_REQUEST_ARC_END";
    case ::CEC::CEC_OPCODE_END_ARC:
      return "CEC_OPCODE_END_ARC";
    case ::CEC::CEC_OPCODE_CDC:
      return "CEC_OPCODE_CDC";
    case ::CEC::CEC_OPCODE_NONE:
      return "CEC_OPCODE_NONE";
    default:
      break;
  }

  return "unknown (" + UTILS::StringUtils::ToHexString(static_cast<uint8_t>(opcode)) + ")";
}

std::string CecTranslator::TranslateKeyCode(::CEC::cec_user_control_code keycode)
{
  switch (keycode)
  {
    case ::CEC::CEC_USER_CONTROL_CODE_SELECT:
      return "CEC_USER_CONTROL_CODE_SELECT";
    case ::CEC::CEC_USER_CONTROL_CODE_UP:
      return "CEC_USER_CONTROL_CODE_UP";
    case ::CEC::CEC_USER_CONTROL_CODE_DOWN:
      return "CEC_USER_CONTROL_CODE_DOWN";
    case ::CEC::CEC_USER_CONTROL_CODE_LEFT:
      return "CEC_USER_CONTROL_CODE_LEFT";
    case ::CEC::CEC_USER_CONTROL_CODE_RIGHT:
      return "CEC_USER_CONTROL_CODE_RIGHT";
    case ::CEC::CEC_USER_CONTROL_CODE_RIGHT_UP:
      return "CEC_USER_CONTROL_CODE_RIGHT_UP";
    case ::CEC::CEC_USER_CONTROL_CODE_RIGHT_DOWN:
      return "CEC_USER_CONTROL_CODE_RIGHT_DOWN";
    case ::CEC::CEC_USER_CONTROL_CODE_LEFT_UP:
      return "CEC_USER_CONTROL_CODE_LEFT_UP";
    case ::CEC::CEC_USER_CONTROL_CODE_LEFT_DOWN:
      return "CEC_USER_CONTROL_CODE_LEFT_DOWN";
    case ::CEC::CEC_USER_CONTROL_CODE_ROOT_MENU:
      return "CEC_USER_CONTROL_CODE_ROOT_MENU";
    case ::CEC::CEC_USER_CONTROL_CODE_SETUP_MENU:
      return "CEC_USER_CONTROL_CODE_SETUP_MENU";
    case ::CEC::CEC_USER_CONTROL_CODE_CONTENTS_MENU:
      return "CEC_USER_CONTROL_CODE_CONTENTS_MENU";
    case ::CEC::CEC_USER_CONTROL_CODE_FAVORITE_MENU:
      return "CEC_USER_CONTROL_CODE_FAVORITE_MENU";
    case ::CEC::CEC_USER_CONTROL_CODE_EXIT:
      return "CEC_USER_CONTROL_CODE_EXIT";
    case ::CEC::CEC_USER_CONTROL_CODE_TOP_MENU:
      return "CEC_USER_CONTROL_CODE_TOP_MENU";
    case ::CEC::CEC_USER_CONTROL_CODE_DVD_MENU:
      return "CEC_USER_CONTROL_CODE_DVD_MENU";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER_ENTRY_MODE:
      return "CEC_USER_CONTROL_CODE_NUMBER_ENTRY_MODE";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER11:
      return "CEC_USER_CONTROL_CODE_NUMBER11";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER12:
      return "CEC_USER_CONTROL_CODE_NUMBER12";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER0:
      return "CEC_USER_CONTROL_CODE_NUMBER0";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER1:
      return "CEC_USER_CONTROL_CODE_NUMBER1";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER2:
      return "CEC_USER_CONTROL_CODE_NUMBER2";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER3:
      return "CEC_USER_CONTROL_CODE_NUMBER3";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER4:
      return "CEC_USER_CONTROL_CODE_NUMBER4";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER5:
      return "CEC_USER_CONTROL_CODE_NUMBER5";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER6:
      return "CEC_USER_CONTROL_CODE_NUMBER6";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER7:
      return "CEC_USER_CONTROL_CODE_NUMBER7";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER8:
      return "CEC_USER_CONTROL_CODE_NUMBER8";
    case ::CEC::CEC_USER_CONTROL_CODE_NUMBER9:
      return "CEC_USER_CONTROL_CODE_NUMBER9";
    case ::CEC::CEC_USER_CONTROL_CODE_DOT:
      return "CEC_USER_CONTROL_CODE_DOT";
    case ::CEC::CEC_USER_CONTROL_CODE_ENTER:
      return "CEC_USER_CONTROL_CODE_ENTER";
    case ::CEC::CEC_USER_CONTROL_CODE_CLEAR:
      return "CEC_USER_CONTROL_CODE_CLEAR";
    case ::CEC::CEC_USER_CONTROL_CODE_NEXT_FAVORITE:
      return "CEC_USER_CONTROL_CODE_NEXT_FAVORITE";
    case ::CEC::CEC_USER_CONTROL_CODE_CHANNEL_UP:
      return "CEC_USER_CONTROL_CODE_CHANNEL_UP";
    case ::CEC::CEC_USER_CONTROL_CODE_CHANNEL_DOWN:
      return "CEC_USER_CONTROL_CODE_CHANNEL_DOWN";
    case ::CEC::CEC_USER_CONTROL_CODE_PREVIOUS_CHANNEL:
      return "CEC_USER_CONTROL_CODE_PREVIOUS_CHANNEL";
    case ::CEC::CEC_USER_CONTROL_CODE_SOUND_SELECT:
      return "CEC_USER_CONTROL_CODE_SOUND_SELECT";
    case ::CEC::CEC_USER_CONTROL_CODE_INPUT_SELECT:
      return "CEC_USER_CONTROL_CODE_INPUT_SELECT";
    case ::CEC::CEC_USER_CONTROL_CODE_DISPLAY_INFORMATION:
      return "CEC_USER_CONTROL_CODE_DISPLAY_INFORMATION";
    case ::CEC::CEC_USER_CONTROL_CODE_HELP:
      return "CEC_USER_CONTROL_CODE_HELP";
    case ::CEC::CEC_USER_CONTROL_CODE_PAGE_UP:
      return "CEC_USER_CONTROL_CODE_PAGE_UP";
    case ::CEC::CEC_USER_CONTROL_CODE_PAGE_DOWN:
      return "CEC_USER_CONTROL_CODE_PAGE_DOWN";
    case ::CEC::CEC_USER_CONTROL_CODE_POWER:
      return "CEC_USER_CONTROL_CODE_POWER";
    case ::CEC::CEC_USER_CONTROL_CODE_VOLUME_UP:
      return "CEC_USER_CONTROL_CODE_VOLUME_UP";
    case ::CEC::CEC_USER_CONTROL_CODE_VOLUME_DOWN:
      return "CEC_USER_CONTROL_CODE_VOLUME_DOWN";
    case ::CEC::CEC_USER_CONTROL_CODE_MUTE:
      return "CEC_USER_CONTROL_CODE_MUTE";
    case ::CEC::CEC_USER_CONTROL_CODE_PLAY:
      return "CEC_USER_CONTROL_CODE_PLAY";
    case ::CEC::CEC_USER_CONTROL_CODE_STOP:
      return "CEC_USER_CONTROL_CODE_STOP";
    case ::CEC::CEC_USER_CONTROL_CODE_PAUSE:
      return "CEC_USER_CONTROL_CODE_PAUSE";
    case ::CEC::CEC_USER_CONTROL_CODE_RECORD:
      return "CEC_USER_CONTROL_CODE_RECORD";
    case ::CEC::CEC_USER_CONTROL_CODE_REWIND:
      return "CEC_USER_CONTROL_CODE_REWIND";
    case ::CEC::CEC_USER_CONTROL_CODE_FAST_FORWARD:
      return "CEC_USER_CONTROL_CODE_FAST_FORWARD";
    case ::CEC::CEC_USER_CONTROL_CODE_EJECT:
      return "CEC_USER_CONTROL_CODE_EJECT";
    case ::CEC::CEC_USER_CONTROL_CODE_FORWARD:
      return "CEC_USER_CONTROL_CODE_FORWARD";
    case ::CEC::CEC_USER_CONTROL_CODE_BACKWARD:
      return "CEC_USER_CONTROL_CODE_BACKWARD";
    case ::CEC::CEC_USER_CONTROL_CODE_STOP_RECORD:
      return "CEC_USER_CONTROL_CODE_STOP_RECORD";
    case ::CEC::CEC_USER_CONTROL_CODE_PAUSE_RECORD:
      return "CEC_USER_CONTROL_CODE_PAUSE_RECORD";
    case ::CEC::CEC_USER_CONTROL_CODE_ANGLE:
      return "CEC_USER_CONTROL_CODE_ANGLE";
    case ::CEC::CEC_USER_CONTROL_CODE_SUB_PICTURE:
      return "CEC_USER_CONTROL_CODE_SUB_PICTURE";
    case ::CEC::CEC_USER_CONTROL_CODE_VIDEO_ON_DEMAND:
      return "CEC_USER_CONTROL_CODE_VIDEO_ON_DEMAND";
    case ::CEC::CEC_USER_CONTROL_CODE_ELECTRONIC_PROGRAM_GUIDE:
      return "CEC_USER_CONTROL_CODE_ELECTRONIC_PROGRAM_GUIDE";
    case ::CEC::CEC_USER_CONTROL_CODE_TIMER_PROGRAMMING:
      return "CEC_USER_CONTROL_CODE_TIMER_PROGRAMMING";
    case ::CEC::CEC_USER_CONTROL_CODE_INITIAL_CONFIGURATION:
      return "CEC_USER_CONTROL_CODE_INITIAL_CONFIGURATION";
    case ::CEC::CEC_USER_CONTROL_CODE_SELECT_BROADCAST_TYPE:
      return "CEC_USER_CONTROL_CODE_SELECT_BROADCAST_TYPE";
    case ::CEC::CEC_USER_CONTROL_CODE_SELECT_SOUND_PRESENTATION:
      return "CEC_USER_CONTROL_CODE_SELECT_SOUND_PRESENTATION";
    case ::CEC::CEC_USER_CONTROL_CODE_PLAY_FUNCTION:
      return "CEC_USER_CONTROL_CODE_PLAY_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_PAUSE_PLAY_FUNCTION:
      return "CEC_USER_CONTROL_CODE_PAUSE_PLAY_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_RECORD_FUNCTION:
      return "CEC_USER_CONTROL_CODE_RECORD_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_PAUSE_RECORD_FUNCTION:
      return "CEC_USER_CONTROL_CODE_PAUSE_RECORD_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_STOP_FUNCTION:
      return "CEC_USER_CONTROL_CODE_STOP_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_MUTE_FUNCTION:
      return "CEC_USER_CONTROL_CODE_MUTE_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_RESTORE_VOLUME_FUNCTION:
      return "CEC_USER_CONTROL_CODE_RESTORE_VOLUME_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_TUNE_FUNCTION:
      return "CEC_USER_CONTROL_CODE_TUNE_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_SELECT_MEDIA_FUNCTION:
      return "CEC_USER_CONTROL_CODE_SELECT_MEDIA_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_SELECT_AV_INPUT_FUNCTION:
      return "CEC_USER_CONTROL_CODE_SELECT_AV_INPUT_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_SELECT_AUDIO_INPUT_FUNCTION:
      return "CEC_USER_CONTROL_CODE_SELECT_AUDIO_INPUT_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_POWER_TOGGLE_FUNCTION:
      return "CEC_USER_CONTROL_CODE_POWER_TOGGLE_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_POWER_OFF_FUNCTION:
      return "CEC_USER_CONTROL_CODE_POWER_OFF_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_POWER_ON_FUNCTION:
      return "CEC_USER_CONTROL_CODE_POWER_ON_FUNCTION";
    case ::CEC::CEC_USER_CONTROL_CODE_F1_BLUE:
      return "CEC_USER_CONTROL_CODE_F1_BLUE";
    case ::CEC::CEC_USER_CONTROL_CODE_F2_RED:
      return "CEC_USER_CONTROL_CODE_F2_RED";
    case ::CEC::CEC_USER_CONTROL_CODE_F3_GREEN:
      return "CEC_USER_CONTROL_CODE_F3_GREEN";
    case ::CEC::CEC_USER_CONTROL_CODE_F4_YELLOW:
      return "CEC_USER_CONTROL_CODE_F4_YELLOW";
    case ::CEC::CEC_USER_CONTROL_CODE_F5:
      return "CEC_USER_CONTROL_CODE_F5";
    case ::CEC::CEC_USER_CONTROL_CODE_DATA:
      return "CEC_USER_CONTROL_CODE_DATA";
    case ::CEC::CEC_USER_CONTROL_CODE_AN_RETURN:
      return "CEC_USER_CONTROL_CODE_AN_RETURN";
    case ::CEC::CEC_USER_CONTROL_CODE_AN_CHANNELS_LIST:
      return "CEC_USER_CONTROL_CODE_AN_CHANNELS_LIST";
    case ::CEC::CEC_USER_CONTROL_CODE_UNKNOWN:
      return "CEC_USER_CONTROL_CODE_UNKNOWN";
    default:
      break;
  }

  return "unknown (" + UTILS::StringUtils::ToHexString(static_cast<uint8_t>(keycode)) + ")";
}

std::string CecTranslator::TranslateVendorID(::CEC::cec_vendor_id vendorId)
{
  // Values are from CECTypeUtils.h
  switch (vendorId)
  {
    case ::CEC::CEC_VENDOR_SAMSUNG:
      return "Samsung";
    case ::CEC::CEC_VENDOR_LG:
      return "LG";
    case ::CEC::CEC_VENDOR_PANASONIC:
      return "Panasonic";
    case ::CEC::CEC_VENDOR_PIONEER:
      return "Pioneer";
    case ::CEC::CEC_VENDOR_ONKYO:
      return "Onkyo";
    case ::CEC::CEC_VENDOR_YAMAHA:
      return "Yamaha";
    case ::CEC::CEC_VENDOR_PHILIPS:
      return "Philips";
    case ::CEC::CEC_VENDOR_SONY:
      return "Sony";
    case ::CEC::CEC_VENDOR_TOSHIBA:
    case ::CEC::CEC_VENDOR_TOSHIBA2:
      return "Toshiba";
    case ::CEC::CEC_VENDOR_APPLE:
      return "Apple";
    case ::CEC::CEC_VENDOR_AKAI:
      return "Akai";
    case ::CEC::CEC_VENDOR_AOC:
      return "AOC";
    case ::CEC::CEC_VENDOR_BENQ:
      return "Benq";
    case ::CEC::CEC_VENDOR_DAEWOO:
      return "Daewoo";
    case ::CEC::CEC_VENDOR_GRUNDIG:
      return "Grundig";
    case ::CEC::CEC_VENDOR_MEDION:
      return "Medion";
    case ::CEC::CEC_VENDOR_SHARP:
    case ::CEC::CEC_VENDOR_SHARP2:
      return "Sharp";
    case ::CEC::CEC_VENDOR_VIZIO:
    case CEC_VENDOR_VIZIO2:
      return "Vizio";
    case ::CEC::CEC_VENDOR_BROADCOM:
      return "Broadcom";
    case ::CEC::CEC_VENDOR_LOEWE:
      return "Loewe";
    case ::CEC::CEC_VENDOR_DENON:
      return "Denon";
    case ::CEC::CEC_VENDOR_MARANTZ:
      return "Marantz";
    case ::CEC::CEC_VENDOR_HARMAN_KARDON:
    case ::CEC::CEC_VENDOR_HARMAN_KARDON2:
      return "Harman/Kardon";
    case ::CEC::CEC_VENDOR_PULSE_EIGHT:
      return "Pulse Eight";
    case ::CEC::CEC_VENDOR_GOOGLE:
      return "Google";
    default:
      break;
  }

  return "";
}

std::string CecTranslator::TranslateBuildDate(std::time_t buildDate)
{
  char buf[sizeof("2011-10-08T07:07:09Z")];

  strftime(buf, sizeof(buf), "%FT%TZ", std::gmtime(&buildDate));

  // This will work too, if your compiler doesn't support %F or %T:
  //strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&buildDate));

  return buf;
}
