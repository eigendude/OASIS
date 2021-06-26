/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "led/ICommandHandler.h"
//#include "led/LedTypes.h"

#include <memory>
#include <stdint.h>
#include <thread>

namespace OASIS
{
namespace LED
{

class LedServer;
class LedThreadCondition;

/*!
 * \brief A thread to control the LED update loop
 *
 * TODO: Move into generic folder for re-usability
 */
class LedThread
{
public:
  LedThread(LedServer &server);
  ~LedThread();

  /*!
   * \brief Initialize and launch the thread
   */
  void Initialize();

  /*!
   * \brief Deinitialize and join the thread
   */
  void Deinitialize();

  /*!
   * \brief Abort a running thread routine
   */
  void Abort();

private:
  /*!
   * \brief Run the thread routine
   */
  void Process();

  /*!
   * \brief Run the thread routine for a single state
   */
  void RunOnce();

  // Construction parameters
  LedServer &m_server;

  // State parameters
  uint64_t m_elapsedMs = 0;

  // Threading parameters
  std::unique_ptr<std::thread> m_thread;
  std::unique_ptr<LedThreadCondition> m_condition;
};

}
}
