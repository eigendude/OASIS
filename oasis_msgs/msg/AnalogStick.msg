################################################################################
#
#  Copyright (C) 2022-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# An analog stick on a peripheral
#

# The name of the analog stick
string name

# The x coordinate in the closed interval [-1, 1] as the analog stick travels
# from fully left to fully right
float32 x

# The y coordinate in the closed interval [-1, 1] as the analog stick travels
# from fully up to fully down
float32 y
