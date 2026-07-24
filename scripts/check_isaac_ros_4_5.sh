#!/usr/bin/env bash
set -euo pipefail

readonly ISAAC_ROS_VENV="/var/lib/isaac-ros-cli/isaac-ros"
readonly ZED_SDK_ROOT="/usr/local/zed"

fail() {
  printf 'Isaac ROS preflight: ERROR: %s\n' "$*" >&2
  exit 1
}

reject_private_path() {
  local path_value="$1"
  local description="$2"

  case "$path_value" in
    *apt_root* | */.local/share/b2arx_isaac_ros/zed_sdk_*)
      fail "$description contains a retired private overlay path: $path_value"
      ;;
  esac
}

require_package() {
  local package_name="$1"
  local package_prefix

  if ! package_prefix="$(ros2 pkg prefix "$package_name" 2>/dev/null)"; then
    fail "ROS package '$package_name' is not discoverable. Install the official 4.5 dependencies and source their setup files."
  fi
  reject_private_path "$package_prefix" "ROS package '$package_name' prefix"
}

require_version_prefix() {
  local package_name="$1"
  local expected_prefix="$2"
  local actual_version

  if ! actual_version="$(ros2 pkg xml "$package_name" -t version 2>/dev/null)"; then
    fail "Could not read the version of ROS package '$package_name'."
  fi
  [[ "$actual_version" == "$expected_prefix" || "$actual_version" == "$expected_prefix".* ]] ||
    fail "ROS package '$package_name' must be $expected_prefix.x, found $actual_version."
}

require_clean_library() {
  local library_path="$1"
  local library_name="$2"
  local linkage
  local missing_libraries
  local dynamic_section

  [[ -f "$library_path" ]] || fail "$library_name was not found at $library_path."

  if ! linkage="$(ldd -- "$library_path" 2>&1)"; then
    fail "Could not inspect the shared-library dependencies of $library_name: $linkage"
  fi
  missing_libraries="$(awk '/not found/ {sub(/^[[:space:]]*/, ""); print}' <<<"$linkage")"
  [[ -z "$missing_libraries" ]] ||
    fail "$library_name has unresolved shared libraries: $missing_libraries"
  reject_private_path "$linkage" "$library_name dependency resolution"

  if ! dynamic_section="$(readelf -d -- "$library_path" 2>&1)"; then
    fail "Could not inspect the ELF dynamic section of $library_name: $dynamic_section"
  fi
  reject_private_path "$dynamic_section" "$library_name ELF dynamic section"
}

require_exact_linkage() {
  local library_path="$1"
  local library_name="$2"
  local dependency_name="$3"
  local expected_library_path="$4"
  local dynamic_section
  local linkage
  local resolved_library_path
  local resolved_library_realpath
  local expected_library_realpath

  if ! dynamic_section="$(readelf -d -- "$library_path" 2>&1)"; then
    fail "Could not inspect the ELF dynamic section of $library_name: $dynamic_section"
  fi
  [[ "$dynamic_section" == *"Shared library: [$dependency_name]"* ]] ||
    fail "$library_name does not directly link the required dependency $dependency_name."

  if ! linkage="$(ldd -- "$library_path" 2>&1)"; then
    fail "Could not resolve $dependency_name for $library_name: $linkage"
  fi
  if ! resolved_library_path="$(
    awk -v dependency="$dependency_name" '
      $1 == dependency && $2 == "=>" { print $3; found = 1; exit }
      END { if (!found) exit 1 }
    ' <<<"$linkage"
  )"; then
    fail "$library_name directly requires $dependency_name, but ldd did not resolve it."
  fi
  [[ "$resolved_library_path" != "not" ]] ||
    fail "$library_name cannot resolve $dependency_name."

  if ! resolved_library_realpath="$(realpath -e -- "$resolved_library_path" 2>/dev/null)"; then
    fail "$library_name resolves $dependency_name to a missing path: $resolved_library_path"
  fi
  if ! expected_library_realpath="$(realpath -e -- "$expected_library_path" 2>/dev/null)"; then
    fail "The required official library is missing: $expected_library_path"
  fi
  reject_private_path "$resolved_library_realpath" "$library_name dependency $dependency_name"
  [[ "$resolved_library_realpath" == "$expected_library_realpath" ]] ||
    fail "$library_name resolves $dependency_name to $resolved_library_realpath, expected $expected_library_realpath."
}

case "${VIRTUAL_ENV:-}" in
  "$ISAAC_ROS_VENV") ;;
  *) fail "run 'isaac-ros activate' first (VIRTUAL_ENV must be $ISAAC_ROS_VENV)." ;;
esac

if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi

for command_name in ros2 python3 ldd readelf realpath awk grep; do
  command -v "$command_name" >/dev/null 2>&1 ||
    fail "Required command '$command_name' is unavailable in the activated environment."
done

for variable_name in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
  variable_value="${!variable_name:-}"
  reject_private_path "$variable_value" "$variable_name"
done

[[ "${ROS_DISTRO:-}" == "jazzy" ]] ||
  fail "ROS_DISTRO must be jazzy. Source /opt/ros/jazzy/setup.bash after activating Isaac ROS."

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
[[ "$RMW_IMPLEMENTATION" == "rmw_cyclonedds_cpp" ]] ||
  fail "RMW_IMPLEMENTATION must be rmw_cyclonedds_cpp, as used by the Isaac ROS 4.5 Isaac Sim tutorial."
if ! python3 -c 'import ctypes; ctypes.CDLL("librmw_cyclonedds_cpp.so")' \
  >/dev/null 2>&1; then
  fail "Cyclone DDS RMW is selected but librmw_cyclonedds_cpp.so cannot be loaded. Install ros-jazzy-rmw-cyclonedds-cpp."
fi

required_packages=(
  isaac_ros_launch_utils
  isaac_ros_nitros
  isaac_ros_managed_nitros
  isaac_ros_nitros_image_type
  isaac_ros_nitros_camera_info_type
  nvblox_examples_bringup
  nvblox_ros
  nvblox_nav2
  nvblox_ros_python_utils
  nav2_bringup
  nav2_common
  nav2_smac_planner
  nav2_regulated_pure_pursuit_controller
  nav2_route
  nav2_collision_monitor
  opennav_docking
  rmw_cyclonedds_cpp
  zed_wrapper
  zed_components
  zed_msgs
)

for package_name in "${required_packages[@]}"; do
  require_package "$package_name"
done

if ! launch_utils_import_error="$(python3 -c 'import isaac_ros_launch_utils' 2>&1)"; then
  fail "Python cannot import isaac_ros_launch_utils in the active Isaac ROS venv: $launch_utils_import_error"
fi

for package_name in \
  isaac_ros_launch_utils \
  isaac_ros_nitros \
  isaac_ros_managed_nitros \
  isaac_ros_nitros_image_type \
  isaac_ros_nitros_camera_info_type \
  nvblox_examples_bringup \
  nvblox_ros \
  nvblox_nav2 \
  nvblox_ros_python_utils; do
  require_version_prefix "$package_name" "4.5"
done

require_version_prefix nav2_bringup "1.3"
require_version_prefix zed_components "5.4"
require_version_prefix zed_wrapper "5.4"
# zed_wrapper 5.4 uses the separately released zed-ros2-interfaces package;
# Stereolabs' current official interface tag/debian for Jazzy is 5.3.x.
require_version_prefix zed_msgs "5.3"

[[ -d "$ZED_SDK_ROOT" ]] ||
  fail "ZED SDK 5.4 is not installed at $ZED_SDK_ROOT. Install the SDK on the host; private SDK copies are not supported."
if ! zed_sdk_root_realpath="$(realpath -e -- "$ZED_SDK_ROOT" 2>/dev/null)"; then
  fail "Could not resolve the host ZED SDK path $ZED_SDK_ROOT."
fi
reject_private_path "$zed_sdk_root_realpath" "Host ZED SDK"

if [[ -n "${ZED_DIR:-}" ]]; then
  if ! configured_zed_dir="$(realpath -e -- "$ZED_DIR" 2>/dev/null)"; then
    fail "ZED_DIR points to a missing SDK path: $ZED_DIR"
  fi
  [[ "$configured_zed_dir" == "$zed_sdk_root_realpath" ]] ||
    fail "ZED_DIR must resolve to the host SDK at $ZED_SDK_ROOT, found $configured_zed_dir."
fi

zed_version_file="$ZED_SDK_ROOT/zed-config-version.cmake"
[[ -f "$zed_version_file" ]] ||
  fail "ZED SDK version metadata was not found at $zed_version_file."
if ! zed_sdk_version="$(
  awk -F '"' '/^[[:space:]]*set\(PACKAGE_VERSION[[:space:]]+"/ { print $2; found = 1; exit }
    END { if (!found) exit 1 }' "$zed_version_file"
)"; then
  fail "Could not read PACKAGE_VERSION from $zed_version_file."
fi
[[ "$zed_sdk_version" == "5.4" || "$zed_sdk_version" == 5.4.* ]] ||
  fail "ZED SDK must be 5.4.x, found $zed_sdk_version at $ZED_SDK_ROOT."

zed_sdk_library="$ZED_SDK_ROOT/lib/libsl_zed.so"
require_clean_library "$zed_sdk_library" "Host ZED SDK library"

nitros_prefix="$(ros2 pkg prefix isaac_ros_nitros)"
nitros_image_prefix="$(ros2 pkg prefix isaac_ros_nitros_image_type)"
reject_private_path "$nitros_prefix" "isaac_ros_nitros prefix"
reject_private_path "$nitros_image_prefix" "isaac_ros_nitros_image_type prefix"
nitros_library="$nitros_prefix/lib/libisaac_ros_nitros.so"
nitros_image_library="$nitros_image_prefix/lib/libisaac_ros_nitros_image_type.so"
require_clean_library "$nitros_library" "Isaac ROS NITROS core library"
require_clean_library "$nitros_image_library" "Isaac ROS NITROS image type library"

zed_prefix="$(ros2 pkg prefix zed_components)"
zed_library="$zed_prefix/lib/libzed_camera_component.so"
require_clean_library "$zed_library" "ZED camera component"
require_exact_linkage "$zed_library" "ZED camera component" \
  "libsl_zed.so" "$zed_sdk_library"
require_exact_linkage "$zed_library" "ZED camera component" \
  "libisaac_ros_nitros.so" "$nitros_library"
require_exact_linkage "$zed_library" "ZED camera component" \
  "libisaac_ros_nitros_image_type.so" "$nitros_image_library"
if ! grep -a -F -q 'Transport summary: IPC=' "$zed_library"; then
  fail "ZED component does not contain the 5.4 transport check. Rebuild the pinned zed-ros2-wrapper v5.4.x source."
fi

nvblox_prefix="$(ros2 pkg prefix nvblox_ros)"
nvblox_library="$nvblox_prefix/lib/libnvblox_ros_lib.so"
require_clean_library "$nvblox_library" "Isaac ROS Nvblox component"
require_exact_linkage "$nvblox_library" "Isaac ROS Nvblox component" \
  "libisaac_ros_nitros.so" "$nitros_library"
require_exact_linkage "$nvblox_library" "Isaac ROS Nvblox component" \
  "libisaac_ros_nitros_image_type.so" "$nitros_image_library"

printf 'Isaac ROS preflight: PASS\n'
printf '  venv: %s\n' "$VIRTUAL_ENV"
printf '  ROS: %s (%s)\n' "$ROS_DISTRO" "$RMW_IMPLEMENTATION"
printf '  Isaac ROS launch utils/NITROS/Nvblox: release 4.5\n'
printf '  Nav2: Jazzy 1.3.x\n'
printf '  Host ZED SDK: %s (%s)\n' "$zed_sdk_version" "$ZED_SDK_ROOT"
printf '  ZED wrapper: 5.4.x with direct NITROS linkage\n'
printf '  Shared-library dependencies: resolved without private overlays\n'
