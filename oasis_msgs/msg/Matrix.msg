################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Row-major matrix container for covariances and Jacobians
#

# Number of rows in the matrix
uint32 rows

# Number of columns in the matrix
uint32 cols

# Row-major data with length equal to rows multiplied by cols
float64[] data
