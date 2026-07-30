################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the ROS-independent train HUD manager."""

from oasis_control.managers.train_hud_manager import TrainHudManager


def test_initial_slam_view_is_hidden() -> None:
    manager: TrainHudManager = TrainHudManager()

    assert manager.show_slam is False


def test_up_press_toggles_and_release_does_not() -> None:
    manager: TrainHudManager = TrainHudManager()

    assert manager.update_up_button(True) is True
    assert manager.show_slam is True
    assert manager.update_up_button(False) is None
    assert manager.show_slam is True
    assert manager.update_up_button(True) is False
    assert manager.show_slam is False


def test_held_up_button_repeats_and_duplicates_are_ignored() -> None:
    manager: TrainHudManager = TrainHudManager()

    assert manager.update_up_button(True) is True
    assert manager.update_up_button(True) is None
    assert manager.update_up_button(True) is None
    assert manager.show_slam is True


def test_duplicate_release_keeps_next_press_armed() -> None:
    manager: TrainHudManager = TrainHudManager()

    assert manager.update_up_button(False) is None
    assert manager.update_up_button(False) is None
    assert manager.update_up_button(True) is True
