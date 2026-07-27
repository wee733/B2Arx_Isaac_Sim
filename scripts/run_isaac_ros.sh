#!/usr/bin/env bash
# Source the installed upstream workspaces, then launch the thin B2ARX wrappers.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_workspace="${repo_root}/ros_ws"
mode="sim"
build_overlay=false
run_preflight=false
dry_run=false
launch_arguments=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_isaac_ros.sh [sim|real] [OPTIONS] [LAUNCH_ARGUMENTS...]

Options:
  --build       Rebuild b2arx_nav2_bringup before launching.
  --preflight   Run the Isaac ROS dependency check before launching.
  --dry-run     Print the final ros2 launch command.
  -h, --help    Show this help.

Environment:
  ISAAC_ROS_WS      Workspace containing the official zed_wrapper overlay.
                    Default: $HOME/workspaces/isaac_ros-dev
  HESAI_WS          Official Hesai driver workspace used by real mode.
                    Default: $HOME/hesai_ws
  HESAI_CONFIG_FILE Real XT32 vendor config used with start_hesai:=true.
  WRIST_REALSENSE_SERIAL Optional D435i selector; leave unset for one camera.
  ROS_DOMAIN_ID     DDS domain, default 23.

Examples:
  ./scripts/run_isaac_ros.sh sim navigation_mode:=depth use_rviz:=true
  ./scripts/run_isaac_ros.sh sim navigation_mode:=lidar use_rviz:=true
  HESAI_CONFIG_FILE=/path/to/config.yaml \
    ./scripts/run_isaac_ros.sh real navigation_mode:=lidar start_hesai:=true \
      start_wrist_realsense:=true serial_number:=12345678
EOF
}

if (($#)) && [[ "$1" == "sim" || "$1" == "real" ]]; then
  mode="$1"
  shift
fi

while (($#)); do
  case "$1" in
    --build)
      build_overlay=true
      shift
      ;;
    --preflight)
      run_preflight=true
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      launch_arguments+=("$@")
      break
      ;;
    *)
      launch_arguments+=("$1")
      shift
      ;;
  esac
done

source_setup() {
  local setup_file="$1"
  local label="$2"
  if [[ ! -f "${setup_file}" ]]; then
    echo "Missing ${label} setup: ${setup_file}" >&2
    return 1
  fi
  # setup.bash is an upstream-generated environment entrypoint.
  # shellcheck disable=SC1090
  set +u
  source "${setup_file}"
  set -u
}

source_setup "/opt/ros/jazzy/setup.bash" "ROS 2 Jazzy"

isaac_ros_ws="${ISAAC_ROS_WS:-${HOME}/workspaces/isaac_ros-dev}"
source_setup "${isaac_ros_ws}/install/setup.bash" "official ZED/Isaac ROS workspace"

hesai_ws="${HESAI_WS:-${HOME}/hesai_ws}"
if [[ "${mode}" == "real" ]]; then
  if [[ -f "${hesai_ws}/install/setup.bash" ]]; then
    source_setup "${hesai_ws}/install/setup.bash" "official Hesai workspace"
  elif printf '%s\n' "${launch_arguments[@]}" | grep -Fxq 'start_hesai:=true'; then
    echo "Real XT32 was requested, but the Hesai workspace is missing: ${hesai_ws}" >&2
    exit 2
  fi
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

if [[ "${build_overlay}" == true || ! -f "${ros_workspace}/install/local_setup.bash" ]]; then
  command -v colcon >/dev/null 2>&1 || {
    echo "colcon is required to build ${ros_workspace}." >&2
    exit 2
  }
  (
    cd "${ros_workspace}"
    colcon build --symlink-install --packages-select b2arx_nav2_bringup
  )
fi

source_setup "${ros_workspace}/install/local_setup.bash" "B2ARX overlay"

for package_name in zed_wrapper nvblox_examples_bringup nav2_bringup b2arx_nav2_bringup; do
  ros2 pkg prefix "${package_name}" >/dev/null 2>&1 || {
    echo "Required ROS package is not discoverable: ${package_name}" >&2
    exit 2
  }
done

if [[ "${run_preflight}" == true ]]; then
  "${repo_root}/scripts/check_isaac_ros_4_5.sh"
fi

launch_file="bringup_${mode}.launch.py"
command=(
  ros2 launch b2arx_nav2_bringup "${launch_file}"
  "domain_id:=${ROS_DOMAIN_ID}"
  "${launch_arguments[@]}"
)

echo "[INFO] mode=${mode} ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[INFO] Isaac ROS/ZED workspace: ${isaac_ros_ws}"
if [[ "${mode}" == "real" && -f "${hesai_ws}/install/setup.bash" ]]; then
  echo "[INFO] Hesai workspace: ${hesai_ws}"
fi

if [[ "${dry_run}" == true ]]; then
  printf '[DRY-RUN]'
  printf ' %q' "${command[@]}"
  printf '\n'
  exit 0
fi

exec "${command[@]}"
