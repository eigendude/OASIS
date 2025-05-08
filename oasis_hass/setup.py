################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import os
import xml.etree.ElementTree

import setuptools


# Colcon symlinks setup.py to an out-of-source build directory
SCRIPT_PATH: str = os.path.realpath(__file__)
SCRIPT_DIRECTORY: str = os.path.abspath(os.path.dirname(SCRIPT_PATH))
PACKAGE_MANIFEST: str = os.path.join(SCRIPT_DIRECTORY, "package.xml")

tree = xml.etree.ElementTree.parse(PACKAGE_MANIFEST)
root = tree.getroot()

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
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.12",
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
        # Systemd services
        (
            os.path.join("share", PACKAGE_NAME, "systemd"),
            [
                "config/systemd/home-assistant.service",
            ],
        ),
        # mqtt_client configuration
        (
            os.path.join("share", PACKAGE_NAME, "mqtt_client"),
            [
                "config/mqtt_client/mqtt_client_params.yaml",
            ],
        ),
    ],
    install_requires=[
        "oasis_msgs",
        "paho-mqtt",
        "rclpy",
    ],
    entry_points={
        "console_scripts": [
            "hass_bridge = oasis_hass.cli.hass_bridge_cli:main",
        ],
    },
)
