################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from oasis_drivers.nodes import telemetrix_bridge_node


def test_bridge_omits_air_quality_topics() -> None:
    assert not hasattr(telemetrix_bridge_node, "AIR_QUALITY_INDEX_TOPIC")
    assert not hasattr(telemetrix_bridge_node, "CO2_TOPIC")
    assert not hasattr(telemetrix_bridge_node, "EQUIVALENT_CO2_TOPIC")
    assert not hasattr(telemetrix_bridge_node, "TVOC_TOPIC")
