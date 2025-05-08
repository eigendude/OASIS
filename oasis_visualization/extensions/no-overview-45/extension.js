/*
 *  Copyright (C) 2024-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from no-overview under the GPL v3 license.
 *  Copyright (C) fthx
 *  https://extensions.gnome.org/extension/4099/no-overview/
 *
 *  SPDX-License-Identifier: Apache-2.0 AND GPL-3.0-or-later
 *  See the file LICENSE.txt for more information.
 */

import { Extension } from "resource:///org/gnome/shell/extensions/extension.js";
import * as Main from "resource:///org/gnome/shell/ui/main.js";

export default class NoOverviewExtension extends Extension {
  /**
   * @brief Construct an extension object
   *
   * Called once when extension is loaded, not enabled.
   *
   * @param {ExtensionMeta} metadata - An extension meta object
   */
  constructor(metadata) {
    super(metadata);

    console.debug(`Constructing ${this.metadata.name}`);
  }

  /**
   * @brief Function called when extension is enabled
   */
  enable() {
    // If the layout manager is not starting up, do nothing
    if (!Main.layoutManager._startingUp) {
      return;
    }

    // Connect to the 'startup-complete' signal to hide the overview
    // when the startup process is complete
    Main.layoutManager.connectObject(
      "startup-complete",
      () => Main.overview.hide(), // Hide the overview
      this,
    );
  }

  /**
   * @brief Function called when extension is uninstalled, disabled, or when the
   *        screen locks
   */
  disable() {
    // Disconnect all signals connected to this object
    Main.layoutManager.disconnectObject(this);
  }
}
