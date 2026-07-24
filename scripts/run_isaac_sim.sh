#!/usr/bin/env bash
# Thin launcher: simulation profiles become arguments for the existing scene.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
invocation_dir="${PWD}"
scene_script="${repo_root}/scripts/isaac_b2arx_scene.py"
config_helper="${repo_root}/scripts/simulation_config.py"
config_path="${repo_root}/config/simulation/warehouse.yaml"
dry_run=false
extra_arguments=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_isaac_sim.sh [--config FILE] [--dry-run] [SCENE_ARGS...]

Load a YAML profile, then run the existing scripts/isaac_b2arx_scene.py entry.
SCENE_ARGS are appended after profile arguments, so normal scene CLI values
override the YAML (for example: --zed_stream_port 31000 --headless).

Environment overrides:
  ISAACLAB_ROOT         Isaac Lab checkout containing isaaclab.sh
  ISAAC_SIM_PYTHON      Direct Isaac Sim/Isaac Lab Python executable
  B2ARX_CONFIG_PYTHON   Python 3 executable with PyYAML for profile parsing
EOF
}

while (($#)); do
  case "$1" in
    --config)
      if (($# < 2)); then
        echo "--config requires a file path" >&2
        exit 2
      fi
      config_path="$2"
      shift 2
      ;;
    --config=*)
      config_path="${1#*=}"
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    --launcher-help)
      usage
      exit 0
      ;;
    --)
      shift
      extra_arguments+=("$@")
      break
      ;;
    *)
      extra_arguments+=("$1")
      shift
      ;;
  esac
done

if [[ "${config_path}" != /* ]]; then
  config_path="${invocation_dir}/${config_path}"
fi

if [[ -n "${B2ARX_CONFIG_PYTHON:-}" ]]; then
  config_python="${B2ARX_CONFIG_PYTHON}"
elif command -v python3 >/dev/null 2>&1; then
  config_python="$(command -v python3)"
elif command -v python >/dev/null 2>&1; then
  config_python="$(command -v python)"
else
  echo "Python 3 with PyYAML is required to read ${config_path}." >&2
  exit 2
fi

config_output="$("${config_python}" "${config_helper}" --config "${config_path}" --emit lines)"
config_arguments=()
if [[ -n "${config_output}" ]]; then
  mapfile -t config_arguments <<<"${config_output}"
fi
scene_arguments=("${config_arguments[@]}" "${extra_arguments[@]}")

# Keep the process environment aligned with the final (last-wins) scene CLI.
ros_domain_id="${ROS_DOMAIN_ID:-23}"
ros_requested=false
for ((index = 0; index < ${#scene_arguments[@]}; ++index)); do
  argument="${scene_arguments[index]}"
  case "${argument}" in
    --ros2|--nav2)
      ros_requested=true
      ;;
    --ros2_domain_id)
      if ((index + 1 >= ${#scene_arguments[@]})); then
        echo "--ros2_domain_id requires a value" >&2
        exit 2
      fi
      ros_domain_id="${scene_arguments[index + 1]}"
      ;;
    --ros2_domain_id=*)
      ros_domain_id="${argument#*=}"
      ;;
  esac
done
export ROS_DOMAIN_ID="${ros_domain_id}"

launch_command=()
if [[ -n "${ISAAC_SIM_PYTHON:-}" ]]; then
  launch_command=("${ISAAC_SIM_PYTHON}" "${scene_script}")
else
  isaaclab_root="${ISAACLAB_ROOT:-}"
  if [[ -z "${isaaclab_root}" && -x "${HOME}/IsaacLab/isaaclab.sh" ]]; then
    isaaclab_root="${HOME}/IsaacLab"
  fi
  if [[ -n "${isaaclab_root}" && -x "${isaaclab_root}/isaaclab.sh" ]]; then
    launch_command=("${isaaclab_root}/isaaclab.sh" -p "${scene_script}")
  elif command -v python >/dev/null 2>&1; then
    launch_command=("$(command -v python)" "${scene_script}")
  else
    echo "Unable to locate Isaac Lab. Activate the isaaclab environment or set ISAACLAB_ROOT." >&2
    exit 2
  fi
fi

echo "[INFO] Simulation config: $(realpath -m "${config_path}")"
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
if [[ "${dry_run}" == true ]]; then
  printf '[DRY-RUN]'
  printf ' %q' "${launch_command[@]}" "${scene_arguments[@]}"
  printf '\n'
  exit 0
fi

# Match the clean-process requirement used by the upstream ARX Isaac Sim
# launcher. Sourced ROS Python 3.12 or ROS library paths can override Isaac
# Sim 5.1's bundled Jazzy/Python 3.11 dependencies before the bridge starts.
if [[ "${ros_requested}" == true && "${PYTHONPATH:-}" == *"python3.12"* ]]; then
  echo "PYTHONPATH contains Python 3.12, but Isaac Sim 5.1 uses Python 3.11." >&2
  echo "Launch from a clean isaaclab terminal; source ROS only in the Isaac ROS terminal." >&2
  exit 2
fi
if [[ "${ros_requested}" == true && "${LD_LIBRARY_PATH:-}" == *"/opt/ros/"* ]]; then
  echo "LD_LIBRARY_PATH contains a ROS installation that can conflict with Isaac Sim." >&2
  echo "Launch from a clean isaaclab terminal; source ROS only in the Isaac ROS terminal." >&2
  exit 2
fi

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

exec "${launch_command[@]}" "${scene_arguments[@]}"
