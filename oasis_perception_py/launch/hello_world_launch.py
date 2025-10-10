################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch import LaunchDescription
from oasis_perception.launch.perception_descriptions import PerceptionDescriptions


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    PerceptionDescriptions.add_hello_world(ld)

    return ld
