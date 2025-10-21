#!/usr/bin/env python3
"""Convert ROS camera_info YAML files into ORB-SLAM3 configuration files."""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Any, Dict, List, Sequence

try:  # pragma: no cover - optional dependency
    import yaml  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    yaml = None  # type: ignore[assignment]


DEFAULT_ORB_PARAMS = {
    "nFeatures": 2000,
    "scaleFactor": 1.2,
    "nLevels": 12,
    "iniThFAST": 20,
    "minThFAST": 7,
}

DEFAULT_VIEWER_PARAMS = {
    "KeyFrameSize": 0.05,
    "KeyFrameLineWidth": 1.0,
    "GraphLineWidth": 0.9,
    "PointSize": 2.0,
    "CameraSize": 0.08,
    "CameraLineWidth": 3.0,
    "ViewpointX": 0.0,
    "ViewpointY": -0.7,
    "ViewpointZ": -3.5,
    "ViewpointF": 500.0,
}

CONFIG_RELATIVE_PATH = Path("oasis_perception_cpp") / "config"


class CameraCalibrationError(RuntimeError):
    """Raised when the input calibration cannot be converted."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
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
        type=float,
        default=30.0,
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
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite the output file if it already exists.",
    )
    return parser.parse_args()


def sanitize_name(name: str) -> str:
    sanitized = re.sub(r"[^0-9A-Za-z_.-]+", "_", name).strip("._")
    return sanitized or "camera"


def load_camera_info(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        raise CameraCalibrationError(f"Camera info file not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        text = handle.read()

    if yaml is not None:  # type: ignore[truthy-bool]
        data = yaml.safe_load(text)
    else:
        data = simple_yaml_load(text)

    if not isinstance(data, dict):
        raise CameraCalibrationError("Camera info file must contain a mapping at the root.")
    return data


def simple_yaml_load(text: str) -> Dict[str, Any]:
    """Parse the subset of YAML syntax used by ROS camera_info files."""

    def parse_scalar(value: str) -> Any:
        stripped = value.strip()
        if not stripped:
            return ""

        if (stripped.startswith("\"") and stripped.endswith("\"")) or (
            stripped.startswith("'") and stripped.endswith("'")
        ):
            return stripped[1:-1]

        if stripped.startswith("[") and stripped.endswith("]"):
            inner = stripped[1:-1].strip()
            if not inner:
                return []
            return [parse_scalar(part) for part in inner.split(",")]

        lowered = stripped.lower()
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

        indent = len(raw_line) - len(raw_line.lstrip(" "))
        line = raw_line.strip()

        if ":" not in line:
            continue

        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()

        while indent <= indent_stack[-1] and len(stack) > 1:
            stack.pop()
            indent_stack.pop()

        current = stack[-1]

        if value == "":
            next_mapping: Dict[str, Any] = {}
            current[key] = next_mapping
            stack.append(next_mapping)
            indent_stack.append(indent)
            continue

        current[key] = parse_scalar(value)

    return root


def extract_intrinsics(
    camera_info: Dict[str, Any], camera_type_override: str | None
) -> tuple[str, Dict[str, Any]]:
    distortion_model = str(camera_info.get("distortion_model", "")).strip().lower()
    matrix = camera_info.get("camera_matrix", {})
    coeffs = camera_info.get("distortion_coefficients", {})

    matrix_data = matrix.get("data") if isinstance(matrix, dict) else None
    if not isinstance(matrix_data, Sequence) or len(matrix_data) != 9:
        raise CameraCalibrationError("camera_matrix.data must contain 9 values.")

    distortion_data = coeffs.get("data") if isinstance(coeffs, dict) else None
    if not isinstance(distortion_data, Sequence) or not distortion_data:
        raise CameraCalibrationError("distortion_coefficients.data must contain values.")

    fx = float(matrix_data[0])
    fy = float(matrix_data[4])
    cx = float(matrix_data[2])
    cy = float(matrix_data[5])

    if camera_type_override:
        camera_type = camera_type_override
    else:
        camera_type = infer_camera_type(distortion_model)

    params: Dict[str, Any]
    if camera_type == "PinHole":
        if len(distortion_data) < 4:
            raise CameraCalibrationError(
                "Pinhole model requires at least four distortion coefficients (k1, k2, p1, p2)."
            )
        k1, k2, p1, p2, *rest = [float(value) for value in distortion_data]
        k3 = float(rest[0]) if rest else None
        params = {
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "k1": k1,
            "k2": k2,
            "p1": p1,
            "p2": p2,
            "k3": k3,
        }
    elif camera_type == "KannalaBrandt8":
        if len(distortion_data) < 4:
            raise CameraCalibrationError(
                "KannalaBrandt8 model requires four distortion coefficients (k1..k4)."
            )
        k1, k2, k3, k4, *_ = [float(value) for value in distortion_data]
        params = {
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "k1": k1,
            "k2": k2,
            "k3": k3,
            "k4": k4,
        }
    elif camera_type == "Rectified":
        params = {
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
        }
    else:
        raise CameraCalibrationError(f"Unsupported camera type: {camera_type}")

    return camera_type, params


def infer_camera_type(distortion_model: str) -> str:
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
    text = f"{value:.8f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    if text == "-0":
        text = "0"
    return text


def build_yaml(
    *,
    camera_type: str,
    params: dict,
    width: int,
    height: int,
    fps: float,
    rgb_flag: int,
    orb_params: dict,
    input_name: str,
) -> str:
    lines: List[str] = ["%YAML:1.0", ""]
    lines.append("#--------------------------------------------------------------------------------------------")
    lines.append("# Camera Parameters generated from")
    lines.append(f"# {input_name}")
    lines.append("#--------------------------------------------------------------------------------------------")
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

    if camera_type in {"PinHole", "KannalaBrandt8"}:
        lines.append("# distortion parameters")
        lines.append(f"Camera1.k1: {format_float(params['k1'])}")
        lines.append(f"Camera1.k2: {format_float(params['k2'])}")
        if camera_type == "PinHole":
            lines.append(f"Camera1.p1: {format_float(params['p1'])}")
            lines.append(f"Camera1.p2: {format_float(params['p2'])}")
            if params.get("k3") is not None:
                lines.append(f"Camera1.k3: {format_float(params['k3'])}")
        else:  # KannalaBrandt8
            lines.append(f"Camera1.k3: {format_float(params['k3'])}")
            lines.append(f"Camera1.k4: {format_float(params['k4'])}")
        lines.append("")

    lines.append("# Camera resolution")
    lines.append(f"Camera.width: {width}")
    lines.append(f"Camera.height: {height}")
    lines.append("")

    lines.append("# Camera frames per second")
    lines.append(f"Camera.fps: {format_float(fps)}")
    lines.append("")

    lines.append(
        "# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)"
    )
    lines.append(f"Camera.RGB: {rgb_flag}")
    lines.append("")

    lines.append("#--------------------------------------------------------------------------------------------")
    lines.append("# ORB Parameters")
    lines.append("#--------------------------------------------------------------------------------------------")
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

    lines.append("#--------------------------------------------------------------------------------------------")
    lines.append("# Viewer Parameters")
    lines.append("#--------------------------------------------------------------------------------------------")
    for key, value in DEFAULT_VIEWER_PARAMS.items():
        lines.append(f"Viewer.{key}: {format_float(value)}")
    lines.append("")

    lines.append('# System.LoadAtlasFromFile: "./atlas"')
    lines.append('# System.SaveAtlasToFile: "./atlas"')

    return "\n".join(lines) + "\n"


def main() -> None:
    args = parse_args()

    camera_info_path = args.camera_info.resolve()
    camera_info = load_camera_info(camera_info_path)

    width = int(camera_info.get("image_width", 0))
    height = int(camera_info.get("image_height", 0))
    if width <= 0 or height <= 0:
        raise CameraCalibrationError("image_width and image_height must be positive integers.")

    camera_type, params = extract_intrinsics(camera_info, args.camera_type)

    orb_params = {
        "nFeatures": args.n_features,
        "scaleFactor": args.scale_factor,
        "nLevels": args.n_levels,
        "iniThFAST": args.init_fast,
        "minThFAST": args.min_fast,
    }

    repo_root = Path(__file__).resolve().parents[2]
    default_output_dir = repo_root / CONFIG_RELATIVE_PATH
    output_dir = args.output_dir or default_output_dir
    output_dir = output_dir.resolve()

    if not output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)

    if args.output_name:
        output_name = sanitize_name(Path(args.output_name).stem)
    else:
        camera_name = str(camera_info.get("camera_name") or camera_info_path.stem)
        output_name = sanitize_name(Path(camera_name).stem)
    output_path = output_dir / f"{output_name}.yaml"

    if output_path.exists() and not args.overwrite:
        raise CameraCalibrationError(
            f"Output file already exists: {output_path}. Use --overwrite to replace it."
        )

    yaml_content = build_yaml(
        camera_type=camera_type,
        params=params,
        width=width,
        height=height,
        fps=args.fps,
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
