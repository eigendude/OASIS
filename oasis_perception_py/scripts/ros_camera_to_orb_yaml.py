#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

"""Convert ROS camera_info YAML files into ORB-SLAM3 configuration files."""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from typing import Dict
from typing import Final
from typing import List
from typing import Literal
from typing import Protocol
from typing import Sequence
from typing import TypedDict
from typing import Union
from typing import cast


class _YamlModule(Protocol):
    def safe_load(self, stream: str) -> Any:  # pragma: no cover - interface definition
        """Parse YAML content and return native Python objects."""


try:  # pragma: no cover - optional dependency
    import yaml as _yaml_module  # type: ignore[import-untyped, import-not-found]
except ImportError:  # pragma: no cover - optional dependency
    yaml_module: _YamlModule | None = None
else:
    yaml_module = cast(_YamlModule, _yaml_module)


class _MatrixDict(TypedDict, total=False):
    data: Sequence[Union[int, float]]


class _CoeffDict(TypedDict, total=False):
    data: Sequence[Union[int, float]]


class CameraInfoDict(TypedDict, total=False):
    distortion_model: str
    camera_matrix: _MatrixDict
    distortion_coefficients: _CoeffDict
    image_width: Union[int, float]
    image_height: Union[int, float]
    camera_name: str


CameraType = Literal["PinHole", "KannalaBrandt8", "Rectified"]


class BaseCameraParams(TypedDict):
    fx: float
    fy: float
    cx: float
    cy: float


class PinHoleParams(BaseCameraParams):
    k1: float
    k2: float
    p1: float
    p2: float
    k3: float | None


class KannalaBrandt8Params(BaseCameraParams):
    k1: float
    k2: float
    k3: float
    k4: float


class RectifiedParams(BaseCameraParams):
    pass


CameraParams = PinHoleParams | KannalaBrandt8Params | RectifiedParams


class OrbParams(TypedDict):
    nFeatures: int
    scaleFactor: float
    nLevels: int
    iniThFAST: int
    minThFAST: int


DEFAULT_ORB_PARAMS: Final[OrbParams] = {
    "nFeatures": 2000,
    "scaleFactor": 1.2,
    "nLevels": 12,
    "iniThFAST": 20,
    "minThFAST": 7,
}

# Suitable for an operator viewing a 50" 1080p display from a few feet away
DEFAULT_VIEWER_PARAMS: Final[Dict[str, float]] = {
    "KeyFrameSize": 0.06,
    "KeyFrameLineWidth": 2.0,
    "GraphLineWidth": 1.4,
    "PointSize": 2.6,
    "CameraSize": 0.10,
    "CameraLineWidth": 3.0,
    "ViewpointX": 0.0,
    "ViewpointY": -0.8,
    "ViewpointZ": -4.5,
    "ViewpointF": 420.0,
}

CONFIG_RELATIVE_PATH: Final[Path] = Path("oasis_perception_cpp") / "config"


class CameraCalibrationError(RuntimeError):
    """Raised when the input calibration cannot be converted."""


@dataclass(frozen=True)
class ParsedArgs:
    camera_info: Path
    output_name: str | None
    output_dir: Path | None
    fps: int
    rgb: int
    camera_type: CameraType | None
    n_features: int
    scale_factor: float
    n_levels: int
    init_fast: int
    min_fast: int


def parse_args() -> ParsedArgs:
    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description=(
            "Generate an ORB-SLAM3 camera configuration from a ROS camera_info YAML file."
        )
    )
    parser.add_argument(
        "camera_info",
        type=Path,
        help="Path to the ROS camera_info YAML file produced by camera calibration.",
    )
    parser.add_argument(
        "--output-name",
        "-o",
        type=str,
        help=(
            "Filename for the generated ORB-SLAM3 YAML (within the config directory). "
            "Defaults to the stem of the input file."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "Optional directory where the ORB-SLAM3 YAML will be written. "
            "Defaults to oasis_perception_cpp/config inside the repository."
        ),
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Camera frame rate used by ORB-SLAM3 (frames per second).",
    )
    parser.add_argument(
        "--rgb",
        type=int,
        choices=(0, 1),
        default=1,
        help="Color order flag for the images (0 for BGR, 1 for RGB).",
    )
    parser.add_argument(
        "--camera-type",
        choices=("PinHole", "KannalaBrandt8", "Rectified"),
        help="Override the inferred ORB-SLAM3 camera type.",
    )
    parser.add_argument(
        "--n-features",
        type=int,
        default=DEFAULT_ORB_PARAMS["nFeatures"],
        help="Number of ORB features to extract per image.",
    )
    parser.add_argument(
        "--scale-factor",
        type=float,
        default=DEFAULT_ORB_PARAMS["scaleFactor"],
        help="Scale factor between pyramid levels in the ORB extractor.",
    )
    parser.add_argument(
        "--n-levels",
        type=int,
        default=DEFAULT_ORB_PARAMS["nLevels"],
        help="Number of pyramid levels in the ORB extractor.",
    )
    parser.add_argument(
        "--init-fast",
        type=int,
        default=DEFAULT_ORB_PARAMS["iniThFAST"],
        help="Initial FAST threshold used by the ORB extractor.",
    )
    parser.add_argument(
        "--min-fast",
        type=int,
        default=DEFAULT_ORB_PARAMS["minThFAST"],
        help="Minimum FAST threshold used when no features are detected.",
    )
    namespace = parser.parse_args()

    return ParsedArgs(
        camera_info=namespace.camera_info,
        output_name=namespace.output_name,
        output_dir=namespace.output_dir,
        fps=namespace.fps,
        rgb=namespace.rgb,
        camera_type=cast(CameraType | None, namespace.camera_type),
        n_features=namespace.n_features,
        scale_factor=namespace.scale_factor,
        n_levels=namespace.n_levels,
        init_fast=namespace.init_fast,
        min_fast=namespace.min_fast,
    )


def sanitize_name(name: str) -> str:
    sanitized: str = re.sub(r"[^0-9A-Za-z_.-]+", "_", name).strip("._")
    return sanitized or "camera"


def load_camera_info(path: Path) -> CameraInfoDict:
    if not path.is_file():
        raise CameraCalibrationError(f"Camera info file not found: {path}")

    text: str
    with path.open("r", encoding="utf-8") as handle:
        text = handle.read()

    data_any: Any
    if yaml_module is not None:
        data_any = yaml_module.safe_load(text)
    else:
        data_any = simple_yaml_load(text)

    if not isinstance(data_any, dict):
        raise CameraCalibrationError(
            "Camera info file must contain a mapping at the root."
        )

    data: CameraInfoDict = data_any  # type: ignore[assignment]
    return data


def simple_yaml_load(text: str) -> Dict[str, Any]:
    """Parse the subset of YAML syntax used by ROS camera_info files."""

    def parse_scalar(value: str) -> Any:
        stripped: str = value.strip()
        if not stripped:
            return ""

        if (stripped.startswith('"') and stripped.endswith('"')) or (
            stripped.startswith("'") and stripped.endswith("'")
        ):
            return stripped[1:-1]

        if stripped.startswith("[") and stripped.endswith("]"):
            inner: str = stripped[1:-1].strip()
            if not inner:
                return []
            return [parse_scalar(part) for part in inner.split(",")]

        lowered: str = stripped.lower()
        if lowered == "true":
            return True
        if lowered == "false":
            return False
        if lowered in {"null", "none"}:
            return None

        try:
            if any(char in stripped for char in (".", "e", "E")):
                return float(stripped)
            return int(stripped)
        except ValueError:
            try:
                return float(stripped)
            except ValueError:
                return stripped

    root: Dict[str, Any] = {}
    stack: List[Dict[str, Any]] = [root]
    indent_stack: List[int] = [-1]

    for raw_line in text.splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue

        indent: int = len(raw_line) - len(raw_line.lstrip(" "))
        line: str = raw_line.strip()

        if ":" not in line:
            continue

        key: str
        value: str
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()

        while indent <= indent_stack[-1] and len(stack) > 1:
            stack.pop()
            indent_stack.pop()

        current: Dict[str, Any] = stack[-1]

        if value == "":
            next_mapping: Dict[str, Any] = {}
            current[key] = next_mapping
            stack.append(next_mapping)
            indent_stack.append(indent)
            continue

        current[key] = parse_scalar(value)

    return root


def extract_intrinsics(
    camera_info: CameraInfoDict, camera_type_override: CameraType | None
) -> tuple[CameraType, CameraParams]:
    distortion_model: str = str(camera_info.get("distortion_model", "")).strip().lower()
    matrix: _MatrixDict = camera_info.get("camera_matrix", {})
    coeffs: _CoeffDict = camera_info.get("distortion_coefficients", {})

    matrix_data: Sequence[float] | None = (
        matrix.get("data") if isinstance(matrix, dict) else None
    )
    if not isinstance(matrix_data, Sequence) or len(matrix_data) != 9:
        raise CameraCalibrationError("camera_matrix.data must contain 9 values.")

    distortion_data: Sequence[float] | None = (
        coeffs.get("data") if isinstance(coeffs, dict) else None
    )
    if not isinstance(distortion_data, Sequence) or not distortion_data:
        raise CameraCalibrationError(
            "distortion_coefficients.data must contain values."
        )

    fx = float(matrix_data[0])
    fy = float(matrix_data[4])
    cx = float(matrix_data[2])
    cy = float(matrix_data[5])

    if camera_type_override:
        camera_type = camera_type_override
    else:
        camera_type = infer_camera_type(distortion_model)

    params: CameraParams
    if camera_type == "PinHole":
        if len(distortion_data) < 4:
            raise CameraCalibrationError(
                "Pinhole model requires at least four distortion coefficients (k1, k2, p1, p2)."
            )
        k1, k2, p1, p2, *rest = [float(value) for value in distortion_data]
        k3 = float(rest[0]) if rest else None
        params = PinHoleParams(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            k1=k1,
            k2=k2,
            p1=p1,
            p2=p2,
            k3=k3,
        )
    elif camera_type == "KannalaBrandt8":
        if len(distortion_data) < 4:
            raise CameraCalibrationError(
                "KannalaBrandt8 model requires four distortion coefficients (k1..k4)."
            )
        k1, k2, k3, k4, *_ = [float(value) for value in distortion_data]
        params = KannalaBrandt8Params(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            k1=k1,
            k2=k2,
            k3=k3,
            k4=k4,
        )
    elif camera_type == "Rectified":
        params = RectifiedParams(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
        )
    else:
        raise CameraCalibrationError(f"Unsupported camera type: {camera_type}")

    return camera_type, params


def infer_camera_type(distortion_model: str) -> CameraType:
    if distortion_model in {"plumb_bob", "rational_polynomial", "brown-conrady"}:
        return "PinHole"
    if distortion_model in {"equidistant", "kannalabrandt8"}:
        return "KannalaBrandt8"
    if distortion_model in {"rectified"}:
        return "Rectified"
    raise CameraCalibrationError(
        "Unable to infer camera type from distortion model: " f"{distortion_model!r}."
    )


def format_float(value: float) -> str:
    text: str = f"{value:.8f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    if text == "-0":
        text = "0"
    if "." not in text and "e" not in text.lower():
        # Ensure values are serialized as floating point numbers in the YAML
        # output. Some downstream parsers (e.g. OpenCV FileStorage) require the
        # value to be tagged as a real number, so we explicitly append a decimal
        # marker when the formatted value looks like an integer.
        text = f"{text}.0"
    return text


def build_yaml(
    *,
    camera_type: CameraType,
    params: CameraParams,
    width: int,
    height: int,
    fps: int,
    rgb_flag: int,
    orb_params: OrbParams,
    input_name: str,
) -> str:
    lines: List[str] = ["%YAML:1.0", ""]
    lines.append(
        "#--------------------------------------------------------------------------------------------"
    )
    lines.append("# Camera Parameters generated from")
    lines.append(f"# {input_name}")
    lines.append(
        "#--------------------------------------------------------------------------------------------"
    )
    lines.append('File.version: "1.0"')
    lines.append("")
    lines.append(f'Camera.type: "{camera_type}"')
    lines.append("")
    lines.append("# Left Camera calibration and distortion parameters (OpenCV)")
    lines.append(f"Camera1.fx: {format_float(params['fx'])}")
    lines.append(f"Camera1.fy: {format_float(params['fy'])}")
    lines.append(f"Camera1.cx: {format_float(params['cx'])}")
    lines.append(f"Camera1.cy: {format_float(params['cy'])}")
    lines.append("")

    if camera_type == "PinHole":
        pinhole_params = cast(PinHoleParams, params)
        lines.append("# distortion parameters")
        lines.append(f"Camera1.k1: {format_float(pinhole_params['k1'])}")
        lines.append(f"Camera1.k2: {format_float(pinhole_params['k2'])}")
        lines.append(f"Camera1.p1: {format_float(pinhole_params['p1'])}")
        lines.append(f"Camera1.p2: {format_float(pinhole_params['p2'])}")
        k3_value = pinhole_params["k3"]
        if k3_value is not None:
            lines.append(f"Camera1.k3: {format_float(k3_value)}")
        lines.append("")
    elif camera_type == "KannalaBrandt8":
        kannala_params = cast(KannalaBrandt8Params, params)
        lines.append("# distortion parameters")
        lines.append(f"Camera1.k1: {format_float(kannala_params['k1'])}")
        lines.append(f"Camera1.k2: {format_float(kannala_params['k2'])}")
        lines.append(f"Camera1.k3: {format_float(kannala_params['k3'])}")
        lines.append(f"Camera1.k4: {format_float(kannala_params['k4'])}")
        lines.append("")

    lines.append("# Camera resolution")
    lines.append(f"Camera.width: {width}")
    lines.append(f"Camera.height: {height}")
    lines.append("")

    lines.append("# Camera frames per second")
    lines.append(f"Camera.fps: {fps}")
    lines.append("")

    lines.append(
        "# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)"
    )
    lines.append(f"Camera.RGB: {rgb_flag}")
    lines.append("")

    lines.append(
        "#--------------------------------------------------------------------------------------------"
    )
    lines.append("# ORB Parameters")
    lines.append(
        "#--------------------------------------------------------------------------------------------"
    )
    lines.append("# ORB Extractor: Number of features per image")
    lines.append(f"ORBextractor.nFeatures: {orb_params['nFeatures']}")
    lines.append("")
    lines.append("# ORB Extractor: Scale factor between levels in the scale pyramid")
    lines.append(f"ORBextractor.scaleFactor: {format_float(orb_params['scaleFactor'])}")
    lines.append("")
    lines.append("# ORB Extractor: Number of levels in the scale pyramid")
    lines.append(f"ORBextractor.nLevels: {orb_params['nLevels']}")
    lines.append("")
    lines.append("# ORB Extractor: Fast threshold")
    lines.append(
        "# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response."
    )
    lines.append(
        "# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST"
    )
    lines.append("# You can lower these values if your images have low contrast")
    lines.append(f"ORBextractor.iniThFAST: {orb_params['iniThFAST']}")
    lines.append(f"ORBextractor.minThFAST: {orb_params['minThFAST']}")
    lines.append("")

    lines.append(
        "#--------------------------------------------------------------------------------------------"
    )
    lines.append("# Viewer Parameters")
    lines.append(
        "#--------------------------------------------------------------------------------------------"
    )
    for key, value in DEFAULT_VIEWER_PARAMS.items():
        lines.append(f"Viewer.{key}: {format_float(value)}")
    lines.append("")

    lines.append('# System.LoadAtlasFromFile: "./atlas"')
    lines.append('# System.SaveAtlasToFile: "./atlas"')

    return "\n".join(lines) + "\n"


def main() -> None:
    args: ParsedArgs = parse_args()

    camera_info_path: Path = args.camera_info.resolve()
    camera_info: CameraInfoDict = load_camera_info(camera_info_path)

    width: int = int(camera_info.get("image_width", 0))
    height: int = int(camera_info.get("image_height", 0))
    if width <= 0 or height <= 0:
        raise CameraCalibrationError(
            "image_width and image_height must be positive integers."
        )

    camera_type: CameraType
    params: CameraParams
    camera_type, params = extract_intrinsics(camera_info, args.camera_type)

    orb_params: OrbParams = {
        "nFeatures": args.n_features,
        "scaleFactor": args.scale_factor,
        "nLevels": args.n_levels,
        "iniThFAST": args.init_fast,
        "minThFAST": args.min_fast,
    }

    repo_root: Path = Path(__file__).resolve().parents[2]
    default_output_dir: Path = repo_root / CONFIG_RELATIVE_PATH
    output_dir: Path = (args.output_dir or default_output_dir).resolve()

    if not output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)

    if args.output_name:
        output_name: str = sanitize_name(Path(args.output_name).stem)
    else:
        camera_name: str = str(
            camera_info.get("camera_name") or camera_info_path.stem
        )
        output_name = sanitize_name(Path(camera_name).stem)
    output_path: Path = output_dir / f"{output_name}.yaml"

    fps_value: int = round(args.fps)
    if not math.isclose(args.fps, fps_value, abs_tol=1e-6):
        raise CameraCalibrationError(
            "Camera FPS must be an integer value compatible with ORB-SLAM3."
        )

    yaml_content: str = build_yaml(
        camera_type=camera_type,
        params=params,
        width=width,
        height=height,
        fps=int(fps_value),
        rgb_flag=args.rgb,
        orb_params=orb_params,
        input_name=str(camera_info.get("camera_name") or camera_info_path.name),
    )

    output_path.write_text(yaml_content, encoding="utf-8")
    print(f"Wrote ORB-SLAM3 configuration to {output_path}")


if __name__ == "__main__":
    try:
        main()
    except CameraCalibrationError as error:
        raise SystemExit(f"Error: {error}") from error
