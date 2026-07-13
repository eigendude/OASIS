/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "RaspberryPiI2cConfig.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
std::string Trim(const std::string& value)
{
  const auto is_not_space = [](unsigned char character) { return !std::isspace(character); };
  const auto begin = std::find_if(value.begin(), value.end(), is_not_space);
  const auto end = std::find_if(value.rbegin(), value.rend(), is_not_space).base();
  return begin < end ? std::string(begin, end) : std::string{};
}

std::vector<std::string> SplitArguments(const std::string& value)
{
  std::vector<std::string> arguments;
  std::istringstream stream(value);
  std::string argument;
  while (std::getline(stream, argument, ','))
    arguments.push_back(Trim(argument));
  return arguments;
}

int ParseAddress(const std::string& text,
                 const std::filesystem::path& config_path,
                 std::size_t line_number)
{
  try
  {
    std::size_t parsed_length = 0;
    const int address = std::stoi(text, &parsed_length, 0);
    if (parsed_length != text.size())
      throw std::invalid_argument("trailing characters");
    return address;
  }
  catch (const std::exception&)
  {
    throw std::runtime_error("Invalid PCA9548 address '" + text + "' in " + config_path.string() +
                             ":" + std::to_string(line_number));
  }
}

std::string HexAddress(int address)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << address;
  return stream.str();
}
} // namespace

OASIS::PowerMeter::RaspberryPiI2cConfig OASIS::PowerMeter::ParseRaspberryPiI2cConfig(
    const std::filesystem::path& config_path, int expected_mux_address)
{
  std::ifstream config_file(config_path);
  if (!config_file)
  {
    throw std::runtime_error("Failed to read Raspberry Pi boot configuration " +
                             config_path.string() + " for PCA9548 address " +
                             HexAddress(expected_mux_address));
  }

  std::vector<int> addresses;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(config_file, line))
  {
    ++line_number;
    const std::size_t comment = line.find('#');
    line = Trim(line.substr(0, comment));
    if (line.empty() || line.front() == '[')
      continue;

    const std::size_t equals = line.find('=');
    if (equals == std::string::npos || Trim(line.substr(0, equals)) != "dtoverlay")
      continue;

    const std::vector<std::string> arguments = SplitArguments(line.substr(equals + 1));
    if (arguments.size() < 2 || arguments[0] != "i2c-mux" || arguments[1] != "pca9548")
      continue;

    std::optional<int> overlay_address;
    for (std::size_t index = 2; index < arguments.size(); ++index)
    {
      const std::size_t argument_equals = arguments[index].find('=');
      if (argument_equals == std::string::npos ||
          Trim(arguments[index].substr(0, argument_equals)) != "addr")
      {
        continue;
      }
      const std::string address_text = Trim(arguments[index].substr(argument_equals + 1));
      overlay_address = ParseAddress(address_text, config_path, line_number);
    }

    if (!overlay_address)
    {
      throw std::runtime_error("PCA9548 overlay in " + config_path.string() + ":" +
                               std::to_string(line_number) + " has no addr argument; expected " +
                               HexAddress(expected_mux_address));
    }
    addresses.push_back(*overlay_address);
  }

  if (addresses.empty())
  {
    throw std::runtime_error("No enabled i2c-mux,pca9548 overlay found in " + config_path.string() +
                             " for address " + HexAddress(expected_mux_address));
  }

  const bool conflicting = std::any_of(addresses.begin(), addresses.end(), [&addresses](int address)
                                       { return address != addresses.front(); });
  if (conflicting)
  {
    throw std::runtime_error("Conflicting PCA9548 overlay addresses in " + config_path.string() +
                             "; expected " + HexAddress(expected_mux_address));
  }
  if (addresses.front() != expected_mux_address)
  {
    throw std::runtime_error("PCA9548 overlay in " + config_path.string() + " uses address " +
                             HexAddress(addresses.front()) + ", but mux_address is " +
                             HexAddress(expected_mux_address));
  }

  return {.mux_address = addresses.front()};
}
