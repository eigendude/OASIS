/*
 *  Copyright (C) 2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from no-overview under the GPL v3 license.
 *  https://github.com/fthx/no-overview
 *
 *  SPDX-License-Identifier: Apache-2.0 AND GPL-3.0-or-later
 *  See the file LICENSE.txt for more information.
 */

import { Extension } from "resource:///org/gnome/shell/extensions/extension.js";
import * as Main from "resource:///org/gnome/shell/ui/main.js";

export default class NoOverviewExtension extends Extension {
  enable() {
    if (!Main.layoutManager._startingUp) {
      return;
    }

    Main.layoutManager.connectObject(
      "startup-complete",
      () => Main.overview.hide(),
      this,
    );
  }

  disable() {
    Main.layoutManager.disconnectObject(this);
  }
}
