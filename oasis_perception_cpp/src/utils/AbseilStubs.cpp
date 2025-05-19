/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include <absl/log/internal/log_message.h>

namespace absl
{
namespace log_internal
{

// We need the compiler to emit code for these two instantiations, as the
// symbols are missing from the monolithic MediaPipe library
template LogMessage& LogMessage::operator<<(const long&);
template LogMessage& LogMessage::operator<<(const unsigned long&);

} // namespace log_internal
} // namespace absl
