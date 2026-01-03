################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import glob
import os
import xml.etree.ElementTree

import setuptools


# Colcon symlinks setup.py to an out-of-source build directory
SCRIPT_PATH = os.path.realpath(__file__)
SCRIPT_DIRECTORY = os.path.abspath(os.path.dirname(SCRIPT_PATH))
PACKAGE_MANIFEST = os.path.join(SCRIPT_DIRECTORY, "package.xml")

tree = xml.etree.ElementTree.parse(PACKAGE_MANIFEST)
root = tree.getroot()

CAMERA_INFO_FILES = sorted(glob.glob(os.path.join("config", "camera_info", "*.yaml")))
SYSTEMD_FILES = sorted(glob.glob(os.path.join("config", "systemd", "*")))

PACKAGE_NAME: str = root.find("name").text  # type: ignore

setuptools.setup(
    name=PACKAGE_NAME,
    version=root.find("version").text,  # type: ignore
    author=root.find("author").text,  # type: ignore
    author_email=root.find("author").attrib["email"],  # type: ignore
    maintainer=root.find("maintainer").text,  # type: ignore
    maintainer_email=root.find("maintainer").attrib["email"],  # type: ignore
    description=root.find("description").text,  # type: ignore
    url=root.find("url").text,  # type: ignore
    license=root.find("license").text,  # type: ignore
    zip_safe=True,
    keywords=[
        "ROS",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Chemistry",
        "Topic :: Scientific/Engineering :: Physics",
    ],
    packages=setuptools.find_packages(exclude=["test"]),
    data_files=[
        # Ament resources
        (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            ["resource/" + PACKAGE_NAME],
        ),
        # Camera info files
        (
            os.path.join("share", PACKAGE_NAME, "camera_info"),
            CAMERA_INFO_FILES,
        ),
        # Launch files
        (
            os.path.join("share", PACKAGE_NAME),
            [
                "launch/drivers_launch.py",
                "launch/ups_launch.py",
            ],
        ),
        # Systemd services
        (
            os.path.join("share", PACKAGE_NAME, "systemd"),
            SYSTEMD_FILES,
        ),
        # udev rules
        (
            os.path.join("share", PACKAGE_NAME, "udev"),
            [
                "config/udev/60-nut-ups.rules",
            ],
        ),
    ],
    install_requires=[
        "psutil",
        "pyserial",
        "setuptools",
        "telemetrix-aio",
    ],
    entry_points={
        "console_scripts": [
            "display_server = oasis_drivers.cli.display_server_cli:main",
            "firmata_bridge = oasis_drivers.cli.firmata_bridge_cli:main",
            "serial_scanner = oasis_drivers.cli.serial_scanner_cli:main",
            "system_monitor = oasis_drivers.cli.system_monitor_cli:main",
            "telemetrix_bridge = oasis_drivers.cli.telemetrix_bridge_cli:main",
            "ups_server = oasis_drivers.cli.ups_server_cli:main",
            "wol_server = oasis_drivers.cli.wol_server_cli:main",
            "train_localization = oasis_drivers.cli.train_localization_cli:main",
        ],
    },
    scripts=[
        "scripts/delay_launch.sh",
        "scripts/wifi_connect.sh",
        "scripts/wifi_disconnect.sh",
        "scripts/wifi_list.sh",
    ],
)
