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
  enable() {
    this._hideCursor = GLib.timeout_add_seconds(
      GLib.PRIORITY_DEFAULT,
      HIDE_CURSOR_TIMEOUT_SECONDS,
      () => {
        let tracker = Meta.CursorTracker.get_for_display(global.display);
        const seat = Clutter.get_default_backend().get_default_seat();

        if (!seat.is_unfocus_inhibited()) seat.inhibit_unfocus();
        tracker.set_pointer_visible(false);

        return GLib.SOURCE_CONTINUE;
      },
    );
  }

  disable() {
    if (this._hideCursor) {
      GLib.Source.remove(this._hideCursor);
      this._hideCursor = null;
    }
    let tracker = Meta.CursorTracker.get_for_display(global.display);
    const seat = Clutter.get_default_backend().get_default_seat();

    if (seat.is_unfocus_inhibited()) seat.uninhibit_unfocus();
    tracker.set_pointer_visible(true);
  }
}
