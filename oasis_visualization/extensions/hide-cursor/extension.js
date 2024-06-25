/*
 *  Copyright (C) 2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Hide Cursor under the GPL v2 license.
 *  Copyright (C) 2024 Alexander Browne
 *  Copyright (C) 2020 Evan Welsh
 *  Copyright (C) 2020 Jeff Channell
 *  https://extensions.gnome.org/extension/6727/hide-cursor/
 *
 *  SPDX-License-Identifier: Apache-2.0 AND GPL-2.0-or-later
 *  See the file LICENSE.txt for more information.
 */

import Clutter from "gi://Clutter";
import GLib from "gi://GLib";
import Meta from "gi://Meta";

import { Extension } from "resource:///org/gnome/shell/extensions/extension.js";

// Note: This initial version has a hardcoded 5 second timeout
const HIDE_CURSOR_TIMEOUT_SECONDS = 5;

export default class HideCursor extends Extension {
  /**
   * @brief Construct an extension object
   *
   * Called once when extension is loaded, not enabled.
   *
   * @param {ExtensionMeta} metadata - An extension meta object
   */
  constructor(metadata) {
    super(metadata);

    console.debug(`constructing ${this.metadata.name}`);
  }

  /**
   * @brief Function called when extension is enabled
   */
  enable() {
    // Set a timeout to hide the cursor after the specified period of inactivity
    this._hideCursor = GLib.timeout_add_seconds(
      GLib.PRIORITY_DEFAULT,
      HIDE_CURSOR_TIMEOUT_SECONDS,
      () => {
        // Get the cursor tracker for the display
        let tracker = Meta.CursorTracker.get_for_display(global.display);

        // Get the default seat from Clutter
        const seat = Clutter.get_default_backend().get_default_seat();

        // Inhibit unfocus if it is not already inhibited
        if (!seat.is_unfocus_inhibited()) {
          seat.inhibit_unfocus();
        }

        // Hide the cursor
        tracker.set_pointer_visible(false);

        // Continue the timeout
        return GLib.SOURCE_CONTINUE;
      },
    );
  }

  /**
   * @brief Function called when extension is uninstalled, disabled, or when the
   *        screen locks
   */
  disable() {
    // Remove the timeout to stop hiding the cursor
    if (this._hideCursor) {
      GLib.Source.remove(this._hideCursor);
      this._hideCursor = null;
    }

    // Get the cursor tracker for the display
    let tracker = Meta.CursorTracker.get_for_display(global.display);

    // Get the default seat from Clutter
    const seat = Clutter.get_default_backend().get_default_seat();

    // Uninhibit unfocus if it is inhibited
    if (seat.is_unfocus_inhibited()) seat.uninhibit_unfocus();

    // Show the cursor
    tracker.set_pointer_visible(true);
  }
}
