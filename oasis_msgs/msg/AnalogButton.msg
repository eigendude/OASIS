################################################################################
#
#  Copyright (C) 2022-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# An analog button on a peripheral. This includes triggers and pressure-
# sensitive buttons.
#

# The name of the button
string name

# The state of the feature, 0.0 for fully unpressed, 1.0 for fully pressed
float32 magnitude
