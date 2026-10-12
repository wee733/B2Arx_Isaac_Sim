"""Shared building blocks for the B2ARX launch files.

Every launch file in this package composes upstream launch APIs. This module
owns the pieces those files previously re-declared one by one: launch-argument
groups, project/upstream include helpers, the strict launch boolean parser, and
the single Nav2 parameter-file preparation step (base file + optional overlay +
behavior-tree rewrite).

Only ``launch``/``launch_ros`` and PyYAML are imported here. Upstream package
lookups (``FindPackageShare``) and upstream Python constants are resolved
lazily at launch time, so the launch descriptions can be built and inspected
without the Isaac ROS / Nav2 overlays on the Python path.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
import copy
import functools
import importlib
import tempfile
from typing import Any

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import (
    EnvironmentVariable,
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.substitutions import FindPackageShare
import yaml


PACKAGE_NAME = "b2arx_nav2_bringup"
NAVIGATION_STACK_LAUNCH = "isaac_ros_nav2.launch.py"

SENSOR_MODES = ("sim", "real")
NAVIGATION_MODES = ("depth", "lidar")
BOOLEAN_CHOICES = ("true", "false")

NAV2_STANDALONE_CONTAINER_NAME = "nav2_container"
NAV2_PARAMS_FILE = "b2arx_nav2.yaml"
XT32_NAV2_OVERLAY_FILE = "b2arx_nav2_xt32_overlay.yaml"
NAV_TO_POSE_BT_XML_KEY = ("bt_navigator", "ros__parameters", "default_nav_to_pose_bt_xml")

_TRUE_WORDS = frozenset({"1", "true", "yes", "on"})
_FALSE_WORDS = frozenset({"0", "false", "no", "off"})


# --------------------------------------------------------------------------
# Booleans and substitutions
# --------------------------------------------------------------------------


def launch_bool(value: str, name: str = "value") -> bool:
    """Parse a launch-argument boolean, rejecting anything ambiguous."""

    normalized = str(value).strip().lower()
    if normalized in _TRUE_WORDS:
        return True
    if normalized in _FALSE_WORDS:
        return False
    raise ValueError(f"{name} must be a boolean value, got {value!r}.")


def launch_config_bool(context: LaunchContext, name: str) -> bool:
    """Evaluate the launch configuration ``name`` with :func:`launch_bool`."""

    return launch_bool(LaunchConfiguration(name).perform(context), name)


def forward(*names: str) -> dict[str, LaunchConfiguration]:
    """Map each name to the same-named launch configuration for an include."""

    return {name: LaunchConfiguration(name) for name in names}


def config_equals(name: str, value: str) -> EqualsSubstitution:
    return EqualsSubstitution(LaunchConfiguration(name), value)


def is_sim_flag(sensor_mode: Any = None) -> IfElseSubstitution:
    """``"true"`` when ``sensor_mode`` is ``sim``, otherwise ``"false"``."""

    if sensor_mode is None:
        sensor_mode = LaunchConfiguration("sensor_mode")
    return IfElseSubstitution(
        EqualsSubstitution(sensor_mode, "sim"),
        if_value="true",
        else_value="false",
    )


class UpstreamConstant(Substitution):
    """Resolve an upstream Python constant only when the launch runs."""

    def __init__(self, module: str, attribute: str) -> None:
        super().__init__()
        self._module = module
        self._attribute = attribute

    def describe(self) -> str:
        return f"{self._module}.{self._attribute}"

    def perform(self, context: LaunchContext) -> str:
        return str(getattr(importlib.import_module(self._module), self._attribute))


def nvblox_container_name() -> UpstreamConstant:
    """The container name owned by the official Isaac ROS Nvblox launch."""

    return UpstreamConstant(
        "nvblox_ros_python_utils.nvblox_constants", "NVBLOX_CONTAINER_NAME"
    )


# --------------------------------------------------------------------------
# Paths and includes
# --------------------------------------------------------------------------


def package_file(package: str, *parts: str) -> PathJoinSubstitution:
    return PathJoinSubstitution([FindPackageShare(package), *parts])


def project_file(*parts: str) -> PathJoinSubstitution:
    return package_file(PACKAGE_NAME, *parts)


def default_nav_to_pose_bt_xml() -> PathJoinSubstitution:
    return package_file(
        "nav2_bt_navigator", "behavior_trees", "navigate_w_replanning_time.xml"
    )


def include_launch(
    location: Any,
    arguments: Mapping[str, Any] | None = None,
    *,
    condition=None,
) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(location),
        launch_arguments=dict(arguments or {}).items(),
        condition=condition,
    )


def include_project_launch(
    filename: str,
    arguments: Mapping[str, Any] | None = None,
    *,
    condition=None,
) -> IncludeLaunchDescription:
    return include_launch(
        project_file("launch", filename), arguments, condition=condition
    )


# --------------------------------------------------------------------------
# Launch-argument groups
# --------------------------------------------------------------------------

ZED_ARGUMENT_NAMES = (
    "sensor_mode",
    "use_sim_time",
    "sim_address",
    "sim_port",
    "serial_number",
    "camera_model",
    "start_zed_wrapper",
    "disable_zed_nitros",
    "zed_params_file",
    "zed_sdk_settings_path",
)
HESAI_ARGUMENT_NAMES = ("start_hesai", "hesai_config_file")
PLATFORM_ADAPTER_ARGUMENT_NAMES = (
    "use_rviz",
    "start_odometry_adapter",
    "publish_zed_mount_tf",
    "publish_hesai_tf",
    "platform_config_file",
)
NAV2_ARGUMENT_NAMES = (
    "start_nav2",
    "autostart",
    "log_level",
    "nav_params_file",
    "nav_params_overlay_file",
    "nav_to_pose_bt_xml",
)


def sensor_mode_argument() -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        "sensor_mode",
        default_value="sim",
        choices=list(SENSOR_MODES),
        description="Select the Isaac Sim stream or a physical ZED camera.",
    )


def sim_time_argument() -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        "use_sim_time",
        default_value=is_sim_flag(),
        description="Defaults to true for sensor_mode:=sim, false for real.",
    )


def zed_arguments() -> list[DeclareLaunchArgument]:
    """Arguments of zed.launch.py, re-declared verbatim by the stack entry."""

    return [
        sensor_mode_argument(),
        sim_time_argument(),
        DeclareLaunchArgument(
            "sim_address",
            default_value=EnvironmentVariable("SIM_ADDRESS", default_value="127.0.0.1"),
            description=(
                "Isaac Sim host's reachable Ethernet address; a CLI value "
                "overrides SIM_ADDRESS."
            ),
        ),
        DeclareLaunchArgument(
            "sim_port",
            default_value=EnvironmentVariable("ZED_PORT", default_value="30000"),
            description="Isaac Sim ZED stream port; a CLI value overrides ZED_PORT.",
        ),
        DeclareLaunchArgument("serial_number", default_value="0"),
        DeclareLaunchArgument("camera_model", default_value="zedx"),
        DeclareLaunchArgument(
            "start_zed_wrapper",
            default_value="true",
            description=(
                "Keep true while ZED VIO owns localization; in lidar mode the "
                "ZED depth is not connected to either Nav2 costmap."
            ),
        ),
        DeclareLaunchArgument("disable_zed_nitros", default_value="true"),
        DeclareLaunchArgument(
            "zed_params_file",
            default_value=project_file("config", "zedx_nvblox_release_4_5.yaml"),
            description="B2ARX overrides layered onto the official ZED configs.",
        ),
        DeclareLaunchArgument(
            "zed_sdk_settings_path",
            default_value=EnvironmentVariable("ZED_SDK_SETTINGS_PATH", default_value=""),
        ),
    ]


def environment_actions() -> list:
    """ROS domain/RMW arguments plus the process environment they drive."""

    return [
        DeclareLaunchArgument("domain_id", default_value="23"),
        DeclareLaunchArgument(
            "rmw_implementation",
            default_value=EnvironmentVariable(
                "RMW_IMPLEMENTATION", default_value="rmw_cyclonedds_cpp"
            ),
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")
        ),
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("LC_ALL", "C"),
    ]


def hesai_arguments() -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "start_hesai",
            default_value="false",
            description=(
                "Start the physical Hesai driver. Keep false in Isaac Sim, "
                "which already publishes /lidar_points."
            ),
        ),
        DeclareLaunchArgument(
            "hesai_config_file",
            default_value=EnvironmentVariable("HESAI_CONFIG_FILE", default_value=""),
            description=(
                "Vendor XT32 YAML. The current real-robot configuration "
                "publishes /lidar_points in frame hesai_lidar."
            ),
        ),
    ]


def platform_adapter_arguments() -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("start_odometry_adapter", default_value="true"),
        DeclareLaunchArgument("publish_zed_mount_tf", default_value="true"),
        DeclareLaunchArgument("publish_hesai_tf", default_value="true"),
        DeclareLaunchArgument(
            "platform_config_file",
            default_value=project_file("config", "platform_adapters.yaml"),
            description="B2ARX frame, sensor mount, and odometry adapter contract.",
        ),
    ]


def nav2_arguments(overlay_default: Any = "") -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument("start_nav2", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument(
            "nav_params_file",
            default_value=project_file("config", NAV2_PARAMS_FILE),
            description="Complete Nav2 parameter file shared by both obstacle modes.",
        ),
        DeclareLaunchArgument(
            "nav_params_overlay_file",
            default_value=overlay_default,
            description=(
                "Optional YAML deep-merged over nav_params_file (a null value "
                "deletes a key). Empty disables the overlay."
            ),
        ),
        DeclareLaunchArgument(
            "nav_to_pose_bt_xml", default_value=default_nav_to_pose_bt_xml()
        ),
    ]


def profile_arguments() -> list[DeclareLaunchArgument]:
    """Arguments shared by the sim and real robot profiles."""

    return [
        DeclareLaunchArgument(
            "navigation_mode",
            default_value="depth",
            choices=list(NAVIGATION_MODES),
            description=(
                "depth: ZED + Isaac ROS Nvblox costmap; "
                "lidar: XT32 PointCloud2 + Nav2 ObstacleLayer."
            ),
        ),
        DeclareLaunchArgument(
            "start_zed_wrapper",
            default_value="true",
            choices=list(BOOLEAN_CHOICES),
            description=(
                "Keep true while ZED VIO owns localization; disable only "
                "when an external stack provides /b2/odom and map TF."
            ),
        ),
        DeclareLaunchArgument(
            "start_odometry_adapter",
            default_value="true",
            choices=list(BOOLEAN_CHOICES),
        ),
    ]


def include_navigation_stack(
    sensor_mode: str, extra_arguments: Mapping[str, Any] | None = None
) -> IncludeLaunchDescription:
    """Include the single navigation stack entry for one robot profile."""

    if sensor_mode not in SENSOR_MODES:
        raise ValueError(f"sensor_mode must be one of {SENSOR_MODES}, got {sensor_mode!r}")
    arguments = {
        "sensor_mode": sensor_mode,
        "use_sim_time": "true" if sensor_mode == "sim" else "false",
        **forward("navigation_mode", "start_zed_wrapper", "start_odometry_adapter"),
        **dict(extra_arguments or {}),
    }
    return include_project_launch(NAVIGATION_STACK_LAUNCH, arguments)


# --------------------------------------------------------------------------
# Nav2 parameter preparation
# --------------------------------------------------------------------------


def deep_merge(base: Mapping, overlay: Mapping) -> dict:
    """Return ``base`` recursively updated by ``overlay``.

    Nested mappings merge, any other overlay value replaces the base value,
    and an overlay value of ``None`` deletes the key.
    """

    merged = copy.deepcopy(dict(base))
    for key, value in overlay.items():
        if value is None:
            merged.pop(key, None)
        elif isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _load_mapping(path: str) -> dict:
    with open(path, encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream)
    if not isinstance(loaded, Mapping):
        raise RuntimeError(f"Nav2 parameter file must contain a mapping: {path}")
    return dict(loaded)


def _set_nested(params: dict, keys: Iterable[str], value: Any) -> None:
    *parents, leaf = tuple(keys)
    node = params
    for key in parents:
        node = node.setdefault(key, {})
    node[leaf] = value


def load_nav2_params(
    base_file: str, overlay_file: str = "", nav_to_pose_bt_xml: str = ""
) -> dict:
    """Merge the base Nav2 file with an optional overlay and BT rewrite."""

    params = _load_mapping(base_file)
    if overlay_file.strip():
        params = deep_merge(params, _load_mapping(overlay_file.strip()))
    if nav_to_pose_bt_xml:
        _set_nested(params, NAV_TO_POSE_BT_XML_KEY, nav_to_pose_bt_xml)
    return params


@functools.lru_cache(maxsize=None)
def write_nav2_params(
    base_file: str, overlay_file: str = "", nav_to_pose_bt_xml: str = ""
) -> str:
    """Write the prepared Nav2 parameters once per distinct input triple."""

    params = load_nav2_params(base_file, overlay_file, nav_to_pose_bt_xml)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="b2arx_nav2_",
        suffix=".yaml",
        delete=False,
    ) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        return stream.name


class Nav2ParamsFile(Substitution):
    """Path of the prepared Nav2 parameter file (see :func:`write_nav2_params`).

    nav2.launch.py and the shared Nvblox container both consume the result.
    Identical inputs resolve to the same file, so the preparation happens once
    per launch even though both consumers perform the substitution.
    """

    def __init__(self, base_file, overlay_file="", nav_to_pose_bt_xml="") -> None:
        super().__init__()
        self._base_file = normalize_to_list_of_substitutions(base_file)
        self._overlay_file = normalize_to_list_of_substitutions(overlay_file)
        self._bt_xml = normalize_to_list_of_substitutions(nav_to_pose_bt_xml)

    def describe(self) -> str:
        return "Nav2ParamsFile(...)"

    def perform(self, context: LaunchContext) -> str:
        return write_nav2_params(
            perform_substitutions(context, self._base_file),
            perform_substitutions(context, self._overlay_file),
            perform_substitutions(context, self._bt_xml),
        )
