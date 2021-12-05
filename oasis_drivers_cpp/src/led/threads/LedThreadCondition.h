/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include <atomic>
#include <stdint.h>

namespace OASIS
{
namespace LED
{

/*!
 * \brief Threading condition variable
 *
 * Currently, notifying the condition variable does not abort the blocking
 * Wait(). This should be changed to use a C++ condition variable.
 */
class LedThreadCondition
{
public:
  LedThreadCondition();

  /*!
   * \brief Wait on condition variable update
   *
   * \param timeoutMs The time to wait, in ms
   *
   * \return True if the variable is set, false otherwise
   */
  bool Wait(int64_t timeoutMs);

  /*!
   * \brief Set a condition variable
   *
   * This causes all asynchronous waits to return.
   */
  void Notify();

private:
  std::atomic<bool> m_valueSet = false;
};

} // namespace LED
} // namespace OASIS
