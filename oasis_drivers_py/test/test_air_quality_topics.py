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


def test_air_quality_topic_names_match_semantic_contract() -> None:
    assert telemetrix_bridge_node.AIR_QUALITY_INDEX_TOPIC == "air_quality_index"
    assert telemetrix_bridge_node.EQUIVALENT_CO2_TOPIC == "equivalent_co2"
    assert telemetrix_bridge_node.TVOC_TOPIC == "tvoc"


def test_co2_topic_is_not_published_by_ens160_path() -> None:
    assert not hasattr(telemetrix_bridge_node, "CO2_TOPIC")


def test_tvoc_ppb_is_converted_to_ppm() -> None:
    assert telemetrix_bridge_node._tvoc_ppb_to_ppm(0) == 0.0
    assert telemetrix_bridge_node._tvoc_ppb_to_ppm(1) == 0.001
    assert telemetrix_bridge_node._tvoc_ppb_to_ppm(1234) == 1.234
