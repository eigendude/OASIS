################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

import os
from typing import List


################################################################################
# Path configuration
################################################################################


# Package names
OASIS_PERCEPTION_CPP_PACKAGE_NAME: str = "oasis_perception_cpp"
ORB_SLAM3_PACKAGE_NAME: str = "ORB_SLAM3"

# ORB_SLAM3 vocabulary file
ORB_SLAM3_VOCABULARY_DIR: str = "Vocabulary"
ORB_SLAM3_VOCABULARY_FILE: str = "ORBvoc.txt"
ORB_SLAM3_VOCABULARY_RELPATH: str = os.path.join(
    "share", ORB_SLAM3_PACKAGE_NAME, ORB_SLAM3_VOCABULARY_DIR, ORB_SLAM3_VOCABULARY_FILE
)

# ORB_SLAM3 settings file
ORB_SLAM3_SETTINGS_DIR: str = "config"
ORB_SLAM3_SETTINGS_FILE: str = (
    "imx708_wide__base_axi_pcie_120000_rp1_i2c_80000_imx708_1a_640x360_4608x2592_BGGR_PISP_COMP1_RAW.yaml"
)
ORB_SLAM3_SETTINGS_RELPATH: str = os.path.join(
    ORB_SLAM3_SETTINGS_DIR, ORB_SLAM3_SETTINGS_FILE
)


################################################################################
# Node configuration
################################################################################


class PerceptionPaths:
    @staticmethod
    def find_orb_slam3_vocabulary() -> str | None:
        """
        Locate the ORB_SLAM3 vocabulary file.

        Returns:
            The full path to the vocabulary file, or None if not found.
        """
        # Scan known prefixes exported by colcon/ament
        prefixes: List[str] = []
        for env_var in ("AMENT_PREFIX_PATH", "COLCON_CURRENT_PREFIX"):
            env_value: str | None = os.getenv(env_var)
            if env_value:
                if env_var == "COLCON_CURRENT_PREFIX":
                    prefixes.append(env_value)
                else:
                    prefixes.extend(
                        [path for path in env_value.split(os.pathsep) if path]
                    )

        for prefix in prefixes:
            candidate: str = os.path.join(prefix, ORB_SLAM3_VOCABULARY_RELPATH)
            if os.path.isfile(candidate):
                return candidate

        return None

    @staticmethod
    def find_orb_slam3_settings() -> str | None:
        """
        Get the path to the camera settings file.

        Returns:
            The full path to the camera settings file, or None if not found
        """
        from ament_index_python import get_package_share_directory

        # Get the package share directory
        share_dir: str = get_package_share_directory(OASIS_PERCEPTION_CPP_PACKAGE_NAME)

        # Construct the full path to the settings file
        settings_path: str = os.path.join(share_dir, (ORB_SLAM3_SETTINGS_RELPATH))

        if os.path.isfile(settings_path):
            return settings_path

        return None
