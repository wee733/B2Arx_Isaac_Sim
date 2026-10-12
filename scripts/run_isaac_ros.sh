#!/usr/bin/env bash
# Source the installed upstream workspaces, then launch the thin B2ARX wrappers.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=lib/common.sh
source "${repo_root}/scripts/lib/common.sh"
B2ARX_LOG_PREFIX="run_isaac_ros"
B2ARX_PACKAGE_HINT="Source the workspace that provides it, or rebuild with --build."

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
  --dry-run     Print the setup files and final ros2 launch command without
                sourcing, building, or launching anything.
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

# setup FILE LABEL: source FILE, or only report it during --dry-run.
setup() {
  if [[ "${dry_run}" == true ]]; then
    local state=""
    [[ -f "$1" ]] || state=" (missing)"
    printf '[DRY-RUN] source %s # %s%s\n' "$1" "$2" "${state}"
    return 0
  fi
  source_setup "$1" "$2"
}

isaac_ros_ws="${ISAAC_ROS_WS:-${HOME}/workspaces/isaac_ros-dev}"
hesai_ws="${HESAI_WS:-${HOME}/hesai_ws}"
hesai_setup="${hesai_ws}/install/setup.bash"
hesai_requested=false
if start_hesai="$(launch_argument_value start_hesai "${launch_arguments[@]}")" &&
   is_launch_true "${start_hesai}"; then
  hesai_requested=true
fi

setup "/opt/ros/jazzy/setup.bash" "ROS 2 Jazzy"
setup "${isaac_ros_ws}/install/setup.bash" "official ZED/Isaac ROS workspace"

use_hesai_ws=false
if [[ "${mode}" == "real" ]]; then
  if [[ -f "${hesai_setup}" ]]; then
    use_hesai_ws=true
  elif [[ "${hesai_requested}" == true ]]; then
    echo "Real XT32 was requested, but the Hesai workspace is missing: ${hesai_ws}" >&2
    [[ "${dry_run}" == true ]] || exit 2
    use_hesai_ws=true
  fi
fi
if [[ "${use_hesai_ws}" == true ]]; then
  setup "${hesai_setup}" "official Hesai workspace"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

build_command=(colcon build --symlink-install --packages-select b2arx_nav2_bringup)
if [[ "${build_overlay}" == true || ! -f "${ros_workspace}/install/local_setup.bash" ]]; then
  if [[ "${dry_run}" == true ]]; then
    printf '[DRY-RUN] (cd %q &&' "${ros_workspace}"
    printf ' %q' "${build_command[@]}"
    printf ')\n'
  else
    command -v colcon >/dev/null 2>&1 || {
      echo "colcon is required to build ${ros_workspace}." >&2
      exit 2
    }
    (cd "${ros_workspace}" && "${build_command[@]}")
  fi
fi

setup "${ros_workspace}/install/local_setup.bash" "B2ARX overlay"

if [[ "${dry_run}" != true ]]; then
  require_packages zed_wrapper nvblox_examples_bringup nav2_bringup b2arx_nav2_bringup
  if [[ "${run_preflight}" == true ]]; then
    "${repo_root}/scripts/check_isaac_ros_4_5.sh"
  fi
elif [[ "${run_preflight}" == true ]]; then
  print_dry_run "${repo_root}/scripts/check_isaac_ros_4_5.sh"
fi

launch_file="bringup_${mode}.launch.py"
command=(
  ros2 launch b2arx_nav2_bringup "${launch_file}"
  "domain_id:=${ROS_DOMAIN_ID}"
  "${launch_arguments[@]}"
)

echo "[INFO] mode=${mode} ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[INFO] Isaac ROS/ZED workspace: ${isaac_ros_ws}"
if [[ "${use_hesai_ws}" == true ]]; then
  echo "[INFO] Hesai workspace: ${hesai_ws}"
fi

if [[ "${dry_run}" == true ]]; then
  print_dry_run "${command[@]}"
  exit 0
fi

exec "${command[@]}"
