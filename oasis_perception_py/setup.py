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

PACKAGE_NAME: str = root.find("name").text  # type: ignore
SYSTEMD_FILES = sorted(glob.glob(os.path.join("config", "systemd", "*")))

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
        # Launch files
        (
            os.path.join("share", PACKAGE_NAME),
            [
                "launch/calibration_launch.py",
                "launch/hello_world_launch.py",
                "launch/perception_launch.py",
            ],
        ),
        # Systemd services
        (
            os.path.join("share", PACKAGE_NAME, "systemd"),
            SYSTEMD_FILES,
        ),
        # MediaPipe files
        (
            os.path.join("share", PACKAGE_NAME, "mediapipe"),
            [
                "mediapipe/pose_landmarker.task",
            ],
        ),
    ],
    install_requires=[
        "mediapipe",
    ],
    entry_points={
        "console_scripts": [
            "camera_calibrator = oasis_perception.cli.camera_calibrator_cli:main",
            "pose_landmarker = oasis_perception.cli.pose_landmarker_cli:main",
            "pose_renderer = oasis_perception.cli.pose_renderer_cli:main",
        ],
    },
)
