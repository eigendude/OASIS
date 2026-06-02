################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from typing import Any
from typing import Sequence

import cv2
import numpy as np
from mediapipe.tasks.python.vision.pose_landmarker import PoseLandmarksConnections
from numpy.typing import NDArray


TASK_POSE_LANDMARK_CONNECTIONS = PoseLandmarksConnections.POSE_LANDMARKS
LANDMARK_COLOR: tuple[int, int, int] = (0, 138, 255)
CONNECTION_COLOR: tuple[int, int, int] = (255, 255, 255)
LANDMARK_RADIUS: int = 4
CONNECTION_THICKNESS: int = 2


def draw_pose_landmarks(
    image: NDArray[np.uint8],
    landmarks: Sequence[Any],
) -> None:
    height: int
    width: int
    height, width = image.shape[:2]

    points: list[tuple[int, int]] = [
        (
            int(round(landmark.x * (width - 1))),
            int(round(landmark.y * (height - 1))),
        )
        for landmark in landmarks
    ]

    for connection in TASK_POSE_LANDMARK_CONNECTIONS:
        if connection.start >= len(points) or connection.end >= len(points):
            continue

        cv2.line(
            image,
            points[connection.start],
            points[connection.end],
            CONNECTION_COLOR,
            CONNECTION_THICKNESS,
        )

    for point in points:
        cv2.circle(
            image,
            point,
            LANDMARK_RADIUS,
            LANDMARK_COLOR,
            thickness=-1,
        )
