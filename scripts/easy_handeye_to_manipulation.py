#!/usr/bin/env python3
"""Convert an easy_handeye eye-in-hand result to the ARX manipulation schema.

easy_handeye2 publishes an eye-in-hand result with ``robot_effector_frame`` as
the TF parent and ``tracking_base_frame`` as the child.  The transform values
therefore pass through unchanged; only an explicit frame alias may be applied
when the same RealSense driver is launched under a new ``camera_name``.
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
import math
from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PROJECT_ROOT / "config" / "arx_r5a_d543if_eih.calib"
DEFAULT_OUTPUT = (
    PROJECT_ROOT
    / "ros_ws"
    / "src"
    / "b2arx_nav2_bringup"
    / "config"
    / "wrist_d435i_eye_in_hand.yaml"
)
DEFAULT_TRACKING_FRAME_ALIASES = {"D543if_link": "wrist_camera_link"}


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a YAML mapping")
    return value


def _frame(value: Any, field: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a non-empty ROS frame id")
    frame = str(value).strip().lstrip("/")
    if not frame or any(character.isspace() for character in frame):
        raise ValueError(f"{field} must be a non-empty ROS frame id")
    return frame


def _numbers(
    values: Mapping[str, Any], keys: Sequence[str], field: str
) -> list[float]:
    result: list[float] = []
    for key in keys:
        value = values.get(key)
        if isinstance(value, bool):
            raise ValueError(f"{field}.{key} must be numeric")
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{field}.{key} must be numeric") from error
        if not math.isfinite(number):
            raise ValueError(f"{field}.{key} must be finite")
        result.append(number)
    return result


def _transform(root: Mapping[str, Any]) -> tuple[list[float], list[float]]:
    raw = _mapping(root.get("transform", root.get("transformation")), "transform")
    if all(key in raw for key in ("x", "y", "z", "qx", "qy", "qz", "qw")):
        return (
            _numbers(raw, ("x", "y", "z"), "transform"),
            _numbers(raw, ("qx", "qy", "qz", "qw"), "transform"),
        )
    translation = _mapping(raw.get("translation"), "transform.translation")
    rotation = _mapping(raw.get("rotation"), "transform.rotation")
    return (
        _numbers(translation, ("x", "y", "z"), "transform.translation"),
        _numbers(rotation, ("x", "y", "z", "w"), "transform.rotation"),
    )


def convert_easy_handeye(
    raw_calibration: Mapping[str, Any],
    *,
    tracking_frame_aliases: Mapping[str, str],
    expected_effector_frame: str = "link6",
) -> dict[str, Any]:
    """Return a schema-v1 static TF config without changing transform direction."""

    root = _mapping(raw_calibration, "calibration")
    parameters = _mapping(root.get("parameters", root), "parameters")
    calibration_type = str(parameters.get("calibration_type", "")).strip()
    if calibration_type != "eye_in_hand":
        raise ValueError(
            "only easy_handeye eye_in_hand calibrations can be converted to a "
            "moving wrist-camera mount"
        )

    effector_frame = _frame(
        parameters.get("robot_effector_frame"), "parameters.robot_effector_frame"
    )
    expected_effector = _frame(expected_effector_frame, "expected_effector_frame")
    if effector_frame != expected_effector:
        raise ValueError(
            f"calibration parent is {effector_frame!r}, expected "
            f"{expected_effector!r}; an additional rigid transform is required"
        )

    source_tracking_frame = _frame(
        parameters.get("tracking_base_frame"), "parameters.tracking_base_frame"
    )
    normalized_aliases = {
        _frame(source, "tracking frame alias source"): _frame(
            target, "tracking frame alias target"
        )
        for source, target in tracking_frame_aliases.items()
    }
    try:
        child_frame = normalized_aliases[source_tracking_frame]
    except KeyError as error:
        raise ValueError(
            f"no explicit frame alias was supplied for tracking_base_frame "
            f"{source_tracking_frame!r}"
        ) from error
    if child_frame == effector_frame:
        raise ValueError("camera child frame must differ from the robot effector frame")

    translation, rotation = _transform(root)
    quaternion_norm = math.sqrt(sum(value * value for value in rotation))
    if not 0.99 <= quaternion_norm <= 1.01:
        raise ValueError(
            f"calibration quaternion norm is {quaternion_norm:.9g}, expected near 1"
        )

    # Do not invert this transform. easy_handeye2's HandeyePublisher publishes
    # the stored value as robot_effector_frame -> tracking_base_frame for
    # eye-in-hand calibrations.
    return {
        "schema_version": 1,
        "publish": True,
        "base_to_camera": {
            "parent_frame": effector_frame,
            "child_frame": child_frame,
            "translation": translation,
            "rotation": rotation,
        },
    }


def load_and_convert(
    source: str | Path,
    *,
    tracking_frame_aliases: Mapping[str, str] = DEFAULT_TRACKING_FRAME_ALIASES,
    expected_effector_frame: str = "link6",
) -> dict[str, Any]:
    source_path = Path(source)
    with source_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    return convert_easy_handeye(
        _mapping(raw, str(source_path)),
        tracking_frame_aliases=tracking_frame_aliases,
        expected_effector_frame=expected_effector_frame,
    )


def render_config(config: Mapping[str, Any], source_label: str) -> str:
    header = (
        "# Generated by scripts/easy_handeye_to_manipulation.py.\n"
        f"# Source: {source_label}\n"
        "# easy_handeye2 eye-in-hand direction is preserved: link6 -> "
        "wrist_camera_link.\n"
        "# The official RealSense driver owns wrist_camera_link -> color optical TF.\n"
    )
    return header + yaml.safe_dump(dict(config), sort_keys=False)


def _alias(value: str) -> tuple[str, str]:
    source, separator, target = value.partition("=")
    if not separator:
        raise argparse.ArgumentTypeError("frame alias must use SOURCE=TARGET")
    return source, target


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--tracking-frame-alias",
        action="append",
        type=_alias,
        metavar="SOURCE=TARGET",
        help=(
            "Explicit old-to-new camera base-frame alias. Defaults to "
            "D543if_link=wrist_camera_link."
        ),
    )
    parser.add_argument("--expected-effector-frame", default="link6")
    parser.add_argument(
        "--check",
        action="store_true",
        help="Fail instead of writing when the generated output is stale.",
    )
    args = parser.parse_args()

    aliases = dict(args.tracking_frame_alias or DEFAULT_TRACKING_FRAME_ALIASES.items())
    config = load_and_convert(
        args.source,
        tracking_frame_aliases=aliases,
        expected_effector_frame=args.expected_effector_frame,
    )
    try:
        source_label = str(args.source.relative_to(PROJECT_ROOT))
    except ValueError:
        source_label = str(args.source)
    rendered = render_config(config, source_label)

    if args.check:
        if not args.output.is_file() or args.output.read_text(encoding="utf-8") != rendered:
            raise SystemExit(f"generated calibration is stale: {args.output}")
        return

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered, encoding="utf-8")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
