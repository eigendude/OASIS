/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "ICommandHandler.h"
#include "LedTypes.h"

#include <map>
#include <memory>
#include <stdint.h>

namespace OASIS
{
namespace LED
{

class LedThread;

/*!
 * \brief Daemon to control LEDs in response to external commands
 *
 * Requires permissions to control LEDs in /sys/class/leds. This can be
 * accomplished with the following udev rule:
 *
 *   SUBSYSTEM=="leds", ACTION=="add", RUN+="/bin/chgrp -R leds /sys%p", RUN+="/bin/chmod -R g=u /sys%p"
 *   SUBSYSTEM=="leds", ACTION=="change", ENV{TRIGGER}!="none", RUN+="/bin/chgrp -R leds /sys%p", RUN+="/bin/chmod -R g=u /sys%p"
 *
 * This will give members of group leds access to all LEDs. Create the leds
 * group now and assign the group to yourself:
 *
 *   sudo groupadd leds
 *   sudo usermod -a -G leds $(whoami)
 *   newgrp leds
 */
class LedServer : public ICommandHandler
{
public:
  LedServer(LedVector);
  virtual ~LedServer();

  // Lifecycle functions
  bool Initialize();
  void Deinitialize();

  // Implementation of ICommandHandler
  void ProcessCommand(const std::string &command) override;

  /*!
   * \brief Update the resources of this class
   *
   * \param runtimeMs The runtime of the process, in ms
   */
  void Update(int64_t runtimeMs);

private:
  using LedCommandMap = std::map<LedCommandType, LedBehaviorPtr>;

  // State variables
  LedCommandType m_currentType = LedCommandType::BUSY_SIGNAL;
  unsigned int m_progressPercent = 0;

  // Resources
  LedCommandMap m_behaviors;

  // Threading variables
  std::unique_ptr<LedThread> m_thread;
};

}
}
