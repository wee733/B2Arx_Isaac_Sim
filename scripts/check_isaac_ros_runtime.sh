#!/usr/bin/env bash
# Live runtime contract check for the B2ARX Isaac ROS navigation stack.
set -euo pipefail

# shellcheck source=lib/common.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"
B2ARX_LOG_PREFIX="Isaac ROS runtime check"

# Interface names. Each can be overridden through the matching B2ARX_* variable.
zed_container="${B2ARX_ZED_CONTAINER:-/zed/zed_container}"
zed_node="${B2ARX_ZED_NODE:-/zed/zed_node}"
nvblox_container="${B2ARX_NVBLOX_CONTAINER:-/nvblox_container}"
nvblox_node="${B2ARX_NVBLOX_NODE:-/nvblox_node}"
nvblox_map_slice_topic="${B2ARX_NVBLOX_MAP_SLICE_TOPIC:-${nvblox_node}/static_map_slice}"
b2_odom_topic="${B2ARX_ODOM_TOPIC:-/b2/odom}"
lidar_topic="${B2ARX_LIDAR_TOPIC:-/lidar_points}"
cmd_vel_topic="${B2ARX_CMD_VEL_TOPIC:-/cmd_vel}"
wrist_namespace="${B2ARX_WRIST_NAMESPACE:-/wrist_camera}"
map_frame="${B2ARX_MAP_FRAME:-map}"
base_frame="${B2ARX_BASE_FRAME:-base_link}"
lidar_frame="${B2ARX_LIDAR_FRAME:-hesai_lidar}"
zed_camera_frame="${B2ARX_ZED_CAMERA_FRAME:-zed_camera_link}"
zed_optical_frame="${B2ARX_ZED_OPTICAL_FRAME:-zed_left_camera_frame_optical}"
wrist_optical_frame="${B2ARX_WRIST_OPTICAL_FRAME:-wrist_camera_color_optical_frame}"

zed_depth_topic="${zed_node}/depth/depth_registered"
zed_color_topic="${zed_node}/rgb/color/rect/image"
zed_camera_info_topic="${zed_node}/depth/camera_info"
zed_pose_topic="${zed_node}/pose"
zed_odom_topic="${zed_node}/odom"
wrist_color_topic="${wrist_namespace}/color/image_raw"
wrist_color_info_topic="${wrist_namespace}/color/camera_info"
wrist_depth_topic="${wrist_namespace}/aligned_depth_to_color/image_raw"
wrist_depth_info_topic="${wrist_namespace}/aligned_depth_to_color/camera_info"
costmap_nodes=(/local_costmap/local_costmap /global_costmap/global_costmap)
lifecycle_nodes=(controller_server planner_server bt_navigator velocity_smoother)

retry_interval_s=0.5

require_wrist_camera=false
navigation_mode=depth

usage() {
  printf 'usage: %s [--navigation-mode depth|lidar] [--require-wrist-camera]\n' "$0"
  printf 'Topic, node, and frame names are overridable with B2ARX_* variables\n'
  printf '(see the top of this script).\n'
}

while (($#)); do
  case "$1" in
    --require-wrist-camera)
      require_wrist_camera=true
      shift
      ;;
    --navigation-mode)
      (($# >= 2)) || fail "--navigation-mode requires depth or lidar"
      navigation_mode="$2"
      shift 2
      ;;
    --navigation-mode=*)
      navigation_mode="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      fail "unknown argument: $1"
      ;;
  esac
done

[[ "$navigation_mode" == depth || "$navigation_mode" == lidar ]] ||
  fail "unsupported navigation mode: $navigation_mode"

command -v ros2 >/dev/null 2>&1 || fail "ros2 is not available in this shell."

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# wait_for_topics ATTEMPTS TOPIC...: fail unless every topic is advertised.
# The topic list is fetched once per attempt for all remaining topics.
wait_for_topics() {
  local attempts="$1"
  local attempt
  local topic_list
  local topic_name
  local -a missing=()
  shift
  missing=("$@")

  for ((attempt = 1; attempt <= attempts; attempt++)); do
    topic_list="$(ros2 topic list 2>/dev/null)" || topic_list=""
    local -a still_missing=()
    for topic_name in "${missing[@]}"; do
      grep -Fxq -- "$topic_name" <<<"$topic_list" || still_missing+=("$topic_name")
    done
    missing=("${still_missing[@]}")
    if ((${#missing[@]} == 0)); then
      return 0
    fi
    sleep "$retry_interval_s"
  done
  fail "required topic is missing: ${missing[*]}"
}

has_publisher() { [[ "$1" =~ Publisher\ count:\ [1-9][0-9]* ]]; }
has_subscriber() { [[ "$1" =~ Subscription\ count:\ [1-9][0-9]* ]]; }

# endpoints_ready INFO TYPE SUBSCRIBER: TYPE may be empty (any type);
# SUBSCRIBER is "none" or a requirement for at least one subscription.
endpoints_ready() {
  local info="$1"
  local type_name="$2"
  local subscriber="$3"

  [[ -z "$type_name" || "$info" == *"Type: $type_name"* ]] &&
    has_publisher "$info" &&
    { [[ "$subscriber" == none ]] || has_subscriber "$info"; }
}

# require_endpoints TOPIC TYPE ATTEMPTS PUBLISHER SUBSCRIBER
#   TYPE        message type, or "" for any
#   PUBLISHER   "any" or the node name that must publish
#   SUBSCRIBER  "none", "any", or the node name that must subscribe
# Retries `ros2 topic info --verbose` until the endpoints exist, then reports
# the first unmet requirement.
require_endpoints() {
  local topic_name="$1"
  local type_name="$2"
  local attempts="$3"
  local publisher="$4"
  local subscriber="$5"
  local info=""
  local attempt

  for ((attempt = 1; attempt <= attempts; attempt++)); do
    info="$(ros2 topic info "$topic_name" --verbose 2>/dev/null)" || info=""
    if endpoints_ready "$info" "$type_name" "$subscriber"; then
      break
    fi
    if ((attempt < attempts)); then
      sleep "$retry_interval_s"
    fi
  done

  [[ -z "$type_name" || "$info" == *"Type: $type_name"* ]] ||
    fail "$topic_name is not $type_name: $info"
  has_publisher "$info" ||
    fail "$topic_name has no publisher (expected: $publisher): $info"
  [[ "$publisher" == any || "$info" == *"Node name: $publisher"* ]] ||
    fail "$topic_name is not published by $publisher: $info"
  [[ "$subscriber" == none ]] && return 0
  has_subscriber "$info" ||
    fail "$topic_name has no subscriber (expected: $subscriber): $info"
  [[ "$subscriber" == any || "$info" == *"Node name: $subscriber"* ]] ||
    fail "$topic_name is not consumed by $subscriber: $info"
}

# require_sole_publisher TOPIC NODE: exactly one publisher, owned by NODE.
require_sole_publisher() {
  local topic_name="$1"
  local node_name="$2"
  local info

  info="$(ros2 topic info "$topic_name" --verbose 2>/dev/null)" ||
    fail "could not inspect $topic_name publishers."
  [[ "$info" == *'Publisher count: 1'$'\n'* || "$info" == *'Publisher count: 1' ]] ||
    fail "$topic_name must have exactly one publisher ($node_name): $info"
  [[ "$info" == *"Node name: $node_name"* ]] ||
    fail "$topic_name is not published exclusively by $node_name: $info"
}

# Message probes run concurrently; each writes its output to a private file.
# Unfinished probes are stopped and the files removed on any exit.
probe_dir="$(mktemp -d)"
declare -A probe_pids=()
cleanup_probes() {
  local pid
  for pid in "${probe_pids[@]}"; do
    kill "$pid" 2>/dev/null || :
  done
  rm -rf -- "$probe_dir"
}
trap cleanup_probes EXIT

# start_echo NAME TIMEOUT_S TOPIC [TYPE [FIELD]]: echo one message. An empty
# TYPE lets ros2 discover it, so the message package need not be importable.
# Without FIELD the message is printed as YAML with arrays truncated.
start_echo() {
  local name="$1"
  local timeout_s="$2"
  local topic_name="$3"
  local type_name="${4:-}"
  local field="${5:-}"
  local -a command=(ros2 topic echo "$topic_name")

  if [[ -n "$type_name" ]]; then
    command+=("$type_name")
  fi
  command+=(--once)
  if [[ -n "$field" ]]; then
    command+=(--field "$field")
  else
    command+=(--truncate-length 64)
  fi
  timeout "${timeout_s}s" "${command[@]}" >"$probe_dir/$name" 2>/dev/null &
  probe_pids[$name]=$!
}

# start_tf NAME TIMEOUT_S PARENT CHILD: capture the first complete tf2_echo
# lookup. tf2_echo is continuous, so stop the pipeline after one transform.
start_tf() {
  local name="$1"
  local timeout_s="$2"
  local parent="$3"
  local child="$4"
  (
    set +o pipefail
    timeout "${timeout_s}s" ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 |
      awk '
        /^At time / { transform_found = 1 }
        transform_found { print }
        transform_found && /^- Rotation: in Quaternion / { exit }
      '
  ) >"$probe_dir/$name" &
  probe_pids[$name]=$!
}

# probe_result NAME FAILURE_MESSAGE: wait for a probe; store its output in
# $probe_output. Must run in this shell (not $(...)), which owns the job.
probe_output=""
probe_result() {
  local name="$1"
  local message="$2"

  wait "${probe_pids[$name]}" || fail "$message"
  unset 'probe_pids[$name]'
  probe_output="$(<"$probe_dir/$name")"
}

# require_tf NAME PARENT CHILD TIMEOUT_S: check a start_tf probe.
require_tf() {
  probe_result "$1" "tf2_echo $2 -> $3 failed"
  { [[ "$probe_output" == *'At time '* ]] &&
    [[ "$probe_output" == *'- Translation: '* ]] &&
    [[ "$probe_output" == *'- Rotation: in Quaternion '* ]]; } ||
    fail "TF $2 -> $3 was not available within $4 seconds: $probe_output"
}

# ---------------------------------------------------------------------------
# Component containers and parameters
# ---------------------------------------------------------------------------

zed_container_components="$(ros2 component list "$zed_container" 2>/dev/null)" ||
  fail "the official $zed_container component manager is not reachable."
[[ "$zed_container_components" == *"$zed_node"* ]] ||
  fail "stereolabs::ZedCamera is not loaded by the official ZED launch."

if [[ "$navigation_mode" == depth ]]; then
  algorithm_container_components="$(ros2 component list "$nvblox_container" 2>/dev/null)" ||
    fail "the $nvblox_container component manager is not reachable."
  [[ "$algorithm_container_components" == *"$nvblox_node"* ]] ||
    fail "nvblox::NvbloxNode is not loaded in $nvblox_container."
fi

nitros_parameter="$(ros2 param get "$zed_node" debug.disable_nitros 2>/dev/null)" ||
  fail "could not read $zed_node debug.disable_nitros."
if [[ "$nitros_parameter" == *True* ]]; then
  zed_transport="sensor_msgs/Image fallback (stable RTX 5090/CUDA 13 default)"
  zed_nitros_enabled=false
elif [[ "$nitros_parameter" == *False* ]]; then
  zed_transport="Managed NITROS (explicit A/B mode)"
  zed_nitros_enabled=true
else
  fail "unexpected debug.disable_nitros value: $nitros_parameter"
fi

if [[ "$navigation_mode" == depth ]]; then
  nvblox_lidar_parameter="$(ros2 param get "$nvblox_node" use_lidar 2>/dev/null)" ||
    fail "could not read $nvblox_node use_lidar."
  [[ "$nvblox_lidar_parameter" == *False* ]] ||
    fail "the supported ZED depth-only Nvblox mode requires use_lidar=false: $nvblox_lidar_parameter"
fi

# ---------------------------------------------------------------------------
# Topics and endpoints
# ---------------------------------------------------------------------------

required_topics=("$zed_pose_topic" "$zed_odom_topic" "$b2_odom_topic" "$lidar_topic")
if [[ "$navigation_mode" == depth ]]; then
  required_topics+=("$zed_depth_topic" "$zed_color_topic" "$nvblox_map_slice_topic")
fi
if [[ "$require_wrist_camera" == true ]]; then
  required_topics+=(
    "$wrist_color_topic"
    "$wrist_color_info_topic"
    "$wrist_depth_topic"
    "$wrist_depth_info_topic"
  )
fi
wait_for_topics 10 "${required_topics[@]}"

# A topic name alone is not proof that the ZED stream is alive: Nvblox's
# subscription endpoints make the base image topics visible even when the ZED
# wrapper has no publisher, and the XT32 can produce a map slice by itself.
# Require both sides of each image path plus the ZED-owned NITROS publishers.
zed_node_name="${zed_node##*/}"
nvblox_node_name="${nvblox_node##*/}"
if [[ "$navigation_mode" == depth ]]; then
  for topic_name in "$zed_depth_topic" "$zed_color_topic"; do
    require_endpoints "$topic_name" sensor_msgs/msg/Image 20 \
      "$zed_node_name" "$nvblox_node_name"
  done
  if [[ "$zed_nitros_enabled" == true ]]; then
    for topic_name in "$zed_depth_topic/nitros" "$zed_color_topic/nitros"; do
      require_endpoints "$topic_name" "" 20 "$zed_node_name" none
    done
  fi
fi

require_endpoints "$b2_odom_topic" nav_msgs/msg/Odometry 1 odometry_adapter none

# XT32 is published independently of ZED Nvblox. Depth mode only verifies its
# publisher; lidar mode additionally requires a Nav2 costmap subscriber.
if [[ "$navigation_mode" == lidar ]]; then
  require_endpoints "$lidar_topic" sensor_msgs/msg/PointCloud2 10 any any
else
  require_endpoints "$lidar_topic" sensor_msgs/msg/PointCloud2 10 any none
fi

if [[ "$require_wrist_camera" == true ]]; then
  for topic_name in "$wrist_color_topic" "$wrist_depth_topic"; do
    require_endpoints "$topic_name" sensor_msgs/msg/Image 20 any none
  done
fi

# ---------------------------------------------------------------------------
# Live messages and TF (independent probes, run concurrently)
# ---------------------------------------------------------------------------

start_echo zed_pose_frame 20 "$zed_pose_topic" geometry_msgs/msg/PoseStamped header.frame_id
start_echo zed_odom_child 20 "$zed_odom_topic" nav_msgs/msg/Odometry child_frame_id
start_echo b2_odom_child 20 "$b2_odom_topic" nav_msgs/msg/Odometry child_frame_id
# One truncated PointCloud2 sample provides both the frame and the fields.
start_echo lidar_cloud 15 "$lidar_topic" sensor_msgs/msg/PointCloud2
start_tf lidar_tf 15 "$base_frame" "$lidar_frame"
# The map -> base lookup traverses the wrapper-owned map -> odom ->
# zed_camera_link edges and this package's zed_camera_link -> base_link
# mounting edge. It therefore catches a dead VIO stream and any accidental TF
# double-parent.
start_tf map_tf 15 "$map_frame" "$base_frame"
if [[ "$navigation_mode" == depth ]]; then
  start_echo zed_camera_info_frame 20 "$zed_camera_info_topic" \
    sensor_msgs/msg/CameraInfo header.frame_id
  start_echo nvblox_map_slice 15 "$nvblox_map_slice_topic"
fi
if [[ "$require_wrist_camera" == true ]]; then
  start_echo wrist_color_frame 20 "$wrist_color_info_topic" \
    sensor_msgs/msg/CameraInfo header.frame_id
  start_echo wrist_depth_frame 20 "$wrist_depth_info_topic" \
    sensor_msgs/msg/CameraInfo header.frame_id
  start_echo wrist_color_encoding 20 "$wrist_color_topic" sensor_msgs/msg/Image encoding
  start_echo wrist_depth_encoding 20 "$wrist_depth_topic" sensor_msgs/msg/Image encoding
fi

if [[ "$navigation_mode" == depth ]]; then
  probe_result zed_camera_info_frame \
    "ZED CameraInfo did not produce a message within 20 seconds."
  zed_camera_frame_id="$probe_output"
  [[ "$zed_camera_frame_id" == *"$zed_optical_frame"* ]] ||
    fail "ZED CameraInfo has the wrong frame_id: $zed_camera_frame_id"
fi

probe_result zed_pose_frame \
  "ZED pose did not produce a message within 20 seconds."
zed_pose_frame_id="$probe_output"
[[ "$zed_pose_frame_id" == *"$map_frame"* ]] ||
  fail "ZED pose must be expressed in $map_frame: $zed_pose_frame_id"

probe_result zed_odom_child \
  "ZED odometry did not produce a message within 20 seconds."
zed_odom_child_frame_id="$probe_output"
[[ "$zed_odom_child_frame_id" == *"$zed_camera_frame"* ]] ||
  fail "ZED odometry child_frame_id must be $zed_camera_frame: $zed_odom_child_frame_id"

probe_result b2_odom_child \
  "B2 odometry did not produce a message within 20 seconds."
b2_odom_child_frame_id="$probe_output"
[[ "$b2_odom_child_frame_id" == *"$base_frame"* ]] ||
  fail "B2 odometry child_frame_id must be $base_frame: $b2_odom_child_frame_id"

probe_result lidar_cloud \
  "$lidar_topic did not produce a PointCloud2 message within 15 seconds."
lidar_cloud="$probe_output"
lidar_frame_id="$(awk '$1 == "frame_id:" { print $2; exit }' <<<"$lidar_cloud")"
[[ "$lidar_frame_id" == "$lidar_frame" ]] ||
  fail "$lidar_topic frame_id must be $lidar_frame: ${lidar_frame_id:-<none>}"
for field_name in x y z; do
  # PointField entries print as YAML list items ("- name: x"); some ros2cli
  # versions quote the value.
  grep -Eq "^-[[:space:]]+name:[[:space:]]*['\"]?${field_name}['\"]?[[:space:]]*$" <<<"$lidar_cloud" ||
    fail "$lidar_topic is missing the $field_name field: $lidar_cloud"
done

require_tf lidar_tf "$base_frame" "$lidar_frame" 15

if [[ "$require_wrist_camera" == true ]]; then
  probe_result wrist_color_frame \
    "wrist color CameraInfo did not produce a message within 20 seconds."
  wrist_color_frame_id="$probe_output"
  [[ "$wrist_color_frame_id" == *"$wrist_optical_frame"* ]] ||
    fail "wrist color CameraInfo has the wrong frame_id: $wrist_color_frame_id"

  probe_result wrist_depth_frame \
    "wrist aligned-depth CameraInfo did not produce a message within 20 seconds."
  wrist_depth_frame_id="$probe_output"
  [[ "$wrist_depth_frame_id" == *"$wrist_optical_frame"* ]] ||
    fail "aligned wrist depth must use the color optical frame: $wrist_depth_frame_id"

  probe_result wrist_color_encoding \
    "wrist color image did not produce a message within 20 seconds."
  wrist_color_encoding="$probe_output"
  [[ "$wrist_color_encoding" == *rgb8* || "$wrist_color_encoding" == *bgr8* ||
     "$wrist_color_encoding" == *rgba8* || "$wrist_color_encoding" == *bgra8* ]] ||
    fail "unsupported wrist color encoding: $wrist_color_encoding"

  probe_result wrist_depth_encoding \
    "wrist aligned-depth image did not produce a message within 20 seconds."
  wrist_depth_encoding="$probe_output"
  if [[ "$wrist_depth_encoding" == *32FC1* ]]; then
    wrist_depth_contract="32FC1 metres (Isaac Sim)"
  elif [[ "$wrist_depth_encoding" == *16UC1* ]]; then
    wrist_depth_contract="16UC1 millimetres / depth_scale 0.001 (RealSense)"
  else
    fail "unsupported wrist aligned-depth encoding: $wrist_depth_encoding"
  fi
fi

require_tf map_tf "$map_frame" "$base_frame" 15

if [[ "$navigation_mode" == depth ]]; then
  probe_result nvblox_map_slice \
    "Nvblox static_map_slice did not produce a message within 15 seconds."
fi

# ---------------------------------------------------------------------------
# Nav2 lifecycle, costmap layers, and command ownership
# ---------------------------------------------------------------------------

for lifecycle_node in "${lifecycle_nodes[@]}"; do
  lifecycle_state="$(ros2 lifecycle get "/$lifecycle_node" 2>/dev/null)" ||
    fail "could not query lifecycle node /$lifecycle_node."
  [[ "$lifecycle_state" == *'active [3]'* ]] ||
    fail "/$lifecycle_node is not active: $lifecycle_state"
done

# In composed Nav2, the controller and planner create their costmap nodes
# dynamically. If the shared component container was not started with the
# complete Nav2 parameter file, those child nodes silently fall back to the
# default static/obstacle layers even though all lifecycle nodes become active.
for costmap_node in "${costmap_nodes[@]}"; do
  costmap_plugins="$(ros2 param get "$costmap_node" plugins 2>/dev/null)" ||
    fail "could not read the plugins parameter from $costmap_node."
  if [[ "$navigation_mode" == depth ]]; then
    { [[ "$costmap_plugins" == *nvblox_layer* ]] &&
      [[ "$costmap_plugins" == *inflation_layer* ]]; } ||
      fail "$costmap_node is not using the Nvblox costmap contract: $costmap_plugins"
    { [[ "$costmap_plugins" != *static_layer* ]] &&
      [[ "$costmap_plugins" != *obstacle_layer* ]]; } ||
      fail "$costmap_node fell back to Nav2 default layers: $costmap_plugins"

    costmap_slice_topic="$(
      ros2 param get "$costmap_node" nvblox_layer.nvblox_map_slice_topic 2>/dev/null
    )" || fail "could not read the Nvblox map slice topic from $costmap_node."
    [[ "$costmap_slice_topic" == *"$nvblox_map_slice_topic"* ]] ||
      fail "$costmap_node uses the wrong Nvblox map slice topic: $costmap_slice_topic"
  else
    { [[ "$costmap_plugins" == *obstacle_layer* ]] &&
      [[ "$costmap_plugins" == *inflation_layer* ]]; } ||
      fail "$costmap_node is not using the XT32 ObstacleLayer contract: $costmap_plugins"
    { [[ "$costmap_plugins" != *nvblox_layer* ]] &&
      [[ "$costmap_plugins" != *static_layer* ]]; } ||
      fail "$costmap_node loaded an unexpected map layer: $costmap_plugins"

    lidar_topic_parameter="$(
      ros2 param get "$costmap_node" obstacle_layer.xt32.topic 2>/dev/null
    )" || fail "could not read the XT32 topic parameter from $costmap_node."
    [[ "$lidar_topic_parameter" == *"$lidar_topic"* ]] ||
      fail "$costmap_node uses the wrong XT32 topic: $lidar_topic_parameter"
  fi
done

# No Nav2 auxiliary server may bypass the policy-aware watchdog.
require_sole_publisher "$cmd_vel_topic" cmd_vel_watchdog

printf 'Isaac ROS runtime check: PASS\n'
printf '  official ZED container: %s\n' "$zed_container"
printf '  ZED image transport: %s\n' "$zed_transport"
printf '  B2 odometry: %s, child %s, lever-arm corrected\n' "$b2_odom_topic" "$base_frame"
printf '  ZED TF: %s -> odom -> %s -> %s\n' "$map_frame" "$zed_camera_frame" "$base_frame"
printf '  XT32 TF: %s -> %s\n' "$base_frame" "$lidar_frame"
if [[ "$require_wrist_camera" == true ]]; then
  printf '  wrist RGB-D: aligned color optical frame, depth %s\n' "$wrist_depth_contract"
fi
if [[ "$navigation_mode" == depth ]]; then
  printf '  obstacle source: ZED depth -> Isaac ROS Nvblox\n'
  printf '  shared algorithm container: Nvblox + composed Nav2\n'
  printf '  ZED depth/color/pose/odom: publishing and connected to Nvblox\n'
  printf '  Nvblox map slice: receiving data\n'
  printf '  Nav2 lifecycle: active; local/global costmaps use Nvblox layers\n'
  printf '  XT32 PointCloud2: %s available independent of ZED Nvblox\n' "$lidar_topic"
else
  printf '  obstacle source: XT32 %s -> Nav2 ObstacleLayer\n' "$lidar_topic"
  printf '  Nav2 execution: standalone official Nav2 processes; Nvblox is not required\n'
  printf '  Nav2 lifecycle: active; local/global costmaps use XT32 ObstacleLayer\n'
fi
printf '  %s publisher: cmd_vel_watchdog only\n' "$cmd_vel_topic"
