################################################################################
#
#  Copyright (C) 2016-2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class InterfaceCallback:
    def interface_added(self, iface):
        """
        @brief A new interface has been discovered
        @param iface The object representing this interface
        """
        pass

    def interface_removed(self, iface):
        """
        @brief An interface is no longer available
        @param iface The object representing this interface
        """
        pass
