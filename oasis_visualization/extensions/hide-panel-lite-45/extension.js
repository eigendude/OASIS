/*
 *  Copyright (C) 2024-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Hide Panel Lite under the GPL v3 license.
 *  Copyright (C) 2021 fthx
 *  https://extensions.gnome.org/extension/4496/hide-panel-light-version-without-hot-corner/
 *
 *  SPDX-License-Identifier: Apache-2.0 AND GPL-3.0-or-later
 *  See the file LICENSE.txt for more information.
 */

import { Extension } from "resource:///org/gnome/shell/extensions/extension.js";
import * as Main from "resource:///org/gnome/shell/ui/main.js";

export default class HidePanelLiteExtension extends Extension {
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
    // Store the original height of the panel to restore it later
    this.panel_height = Main.panel.get_height();

    // Hide the panel initially
    this._hide_panel();

    // Connect signals to show/hide the panel when entering/leaving the overview
    this.showing = Main.overview.connect(
      "showing",
      this._show_panel.bind(this),
    );
    this.hiding = Main.overview.connect("hiding", this._hide_panel.bind(this));
  }

  /**
   * @brief Function called when extension is uninstalled, disabled, or when the
   *        screen locks
   */
  disable() {
    // Disconnect signals to stop showing/hiding the panel
    Main.overview.disconnect(this.showing);
    Main.overview.disconnect(this.hiding);

    // Restore the panel to its original height
    this._show_panel();
  }

  _show_panel() {
    // Set the panel height to the stored original height
    Main.panel.set_height(this.panel_height);
  }

  _hide_panel() {
    // Set the panel height to 0 to hide it
    Main.panel.set_height(0);
  }
}
