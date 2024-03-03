/*
 *  Copyright (C) 2023-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from no-overview under the GPL v3 license.
 *  https://github.com/fthx/no-overview
 *
 *  SPDX-License-Identifier: Apache-2.0 AND GPL-3.0-or-later
 *  See the file LICENSE.txt for more information.
 */

const Main = imports.ui.main;

class NoOverviewExtension {
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

function init() {
  return new NoOverviewExtension();
}
