#!/usr/bin/env bash
set -euo pipefail

fail() {
  printf 'Isaac ROS runtime check: ERROR: %s\n' "$*" >&2
  exit 1
}

require_wrist_camera=false
navigation_mode=depth

usage() {
  printf 'usage: %s [--navigation-mode depth|lidar] [--require-wrist-camera]\n' "$0"
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

zed_container_components="$(ros2 component list /zed/zed_container 2>/dev/null)" ||
  fail "the official /zed/zed_container component manager is not reachable."
[[ "$zed_container_components" == *'/zed/zed_node'* ]] ||
  fail "stereolabs::ZedCamera is not loaded by the official ZED launch."

if [[ "$navigation_mode" == depth ]]; then
  algorithm_container_components="$(ros2 component list /nvblox_container 2>/dev/null)" ||
    fail "the /nvblox_container component manager is not reachable."
  [[ "$algorithm_container_components" == *'/nvblox_node'* ]] ||
    fail "nvblox::NvbloxNode is not loaded in /nvblox_container."
fi

nitros_parameter="$(ros2 param get /zed/zed_node debug.disable_nitros 2>/dev/null)" ||
  fail "could not read /zed/zed_node debug.disable_nitros."
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
  nvblox_lidar_parameter="$(ros2 param get /nvblox_node use_lidar 2>/dev/null)" ||
    fail "could not read /nvblox_node use_lidar."
  [[ "$nvblox_lidar_parameter" == *False* ]] ||
    fail "the supported ZED depth-only Nvblox mode requires use_lidar=false: $nvblox_lidar_parameter"
fi

required_topics=(
  /zed/zed_node/pose
  /zed/zed_node/odom
  /b2/odom
  /lidar_points
)
if [[ "$navigation_mode" == depth ]]; then
  required_topics+=(
    /zed/zed_node/depth/depth_registered
    /zed/zed_node/rgb/color/rect/image
    /nvblox_node/static_map_slice
  )
fi
if [[ "$require_wrist_camera" == true ]]; then
  required_topics+=(
    /wrist_camera/color/image_raw
    /wrist_camera/color/camera_info
    /wrist_camera/aligned_depth_to_color/image_raw
    /wrist_camera/aligned_depth_to_color/camera_info
  )
fi
for topic_name in "${required_topics[@]}"; do
  topic_found=false
  for attempt in $(seq 1 10); do
    topic_list="$(ros2 topic list 2>/dev/null || true)"
    if [[ "$topic_list" == *"$topic_name"* ]]; then
      topic_found=true
      break
    fi
    sleep 0.5
  done
  [[ "$topic_found" == true ]] || fail "required topic is missing: $topic_name"
done

# A topic name alone is not proof that the ZED stream is alive: Nvblox's
# subscription endpoints make the base image topics visible even when the ZED
# wrapper has no publisher, and the XT32 can produce a map slice by itself.
# Require both sides of each image path plus the ZED-owned NITROS publishers.
if [[ "$navigation_mode" == depth ]]; then
  zed_image_topics=(
    /zed/zed_node/depth/depth_registered
    /zed/zed_node/rgb/color/rect/image
  )
  for topic_name in "${zed_image_topics[@]}"; do
    zed_image_info=""
    for attempt in $(seq 1 20); do
      zed_image_info="$(ros2 topic info "$topic_name" --verbose 2>/dev/null || true)"
      if [[ "$zed_image_info" == *'Type: sensor_msgs/msg/Image'* ]] &&
         [[ "$zed_image_info" =~ Publisher\ count:\ [1-9][0-9]* ]] &&
         [[ "$zed_image_info" =~ Subscription\ count:\ [1-9][0-9]* ]]; then
        break
      fi
      sleep 0.5
    done
    [[ "$zed_image_info" == *'Type: sensor_msgs/msg/Image'* ]] ||
      fail "$topic_name is not sensor_msgs/msg/Image: $zed_image_info"
    [[ "$zed_image_info" =~ Publisher\ count:\ [1-9][0-9]* ]] ||
      fail "$topic_name has no ZED publisher: $zed_image_info"
    [[ "$zed_image_info" =~ Subscription\ count:\ [1-9][0-9]* ]] ||
      fail "$topic_name has no Nvblox subscriber: $zed_image_info"
    [[ "$zed_image_info" == *'Node name: zed_node'* ]] ||
      fail "$topic_name is not published by zed_node: $zed_image_info"
    [[ "$zed_image_info" == *'Node name: nvblox_node'* ]] ||
      fail "$topic_name is not consumed by nvblox_node: $zed_image_info"
  done
fi

if [[ "$zed_nitros_enabled" == true ]]; then
  if [[ "$navigation_mode" == depth ]]; then
    zed_nitros_topics=(
      /zed/zed_node/depth/depth_registered/nitros
      /zed/zed_node/rgb/color/rect/image/nitros
    )
    for topic_name in "${zed_nitros_topics[@]}"; do
      zed_nitros_info=""
      for attempt in $(seq 1 20); do
        zed_nitros_info="$(ros2 topic info "$topic_name" --verbose 2>/dev/null || true)"
        if [[ "$zed_nitros_info" =~ Publisher\ count:\ [1-9][0-9]* ]]; then
          break
        fi
        sleep 0.5
      done
      [[ "$zed_nitros_info" =~ Publisher\ count:\ [1-9][0-9]* ]] ||
        fail "$topic_name has no Managed NITROS publisher: $zed_nitros_info"
      [[ "$zed_nitros_info" == *'Node name: zed_node'* ]] ||
        fail "$topic_name is not published by zed_node: $zed_nitros_info"
    done
  fi
fi

if [[ "$navigation_mode" == depth ]]; then
  zed_camera_frame_id="$(
    timeout 20s ros2 topic echo /zed/zed_node/depth/camera_info \
      sensor_msgs/msg/CameraInfo --field header.frame_id --once 2>/dev/null
  )" || fail "ZED CameraInfo did not produce a message within 20 seconds."
  [[ "$zed_camera_frame_id" == *zed_left_camera_frame_optical* ]] ||
    fail "ZED CameraInfo has the wrong frame_id: $zed_camera_frame_id"
fi

zed_pose_frame_id="$(
  timeout 20s ros2 topic echo /zed/zed_node/pose \
    geometry_msgs/msg/PoseStamped --field header.frame_id --once 2>/dev/null
)" || fail "ZED pose did not produce a message within 20 seconds."
[[ "$zed_pose_frame_id" == *map* ]] ||
  fail "ZED pose must be expressed in map: $zed_pose_frame_id"

zed_odom_child_frame_id="$(
  timeout 20s ros2 topic echo /zed/zed_node/odom \
    nav_msgs/msg/Odometry --field child_frame_id --once 2>/dev/null
)" || fail "ZED odometry did not produce a message within 20 seconds."
[[ "$zed_odom_child_frame_id" == *zed_camera_link* ]] ||
  fail "ZED odometry child_frame_id must be zed_camera_link: $zed_odom_child_frame_id"

b2_odom_info="$(ros2 topic info /b2/odom --verbose 2>/dev/null)" ||
  fail "could not inspect the adapted /b2/odom topic."
[[ "$b2_odom_info" == *'Type: nav_msgs/msg/Odometry'* ]] ||
  fail "/b2/odom has the wrong type: $b2_odom_info"
[[ "$b2_odom_info" == *'Node name: odometry_adapter'* ]] ||
  fail "/b2/odom is not published by odometry_adapter: $b2_odom_info"
b2_odom_child_frame_id="$(
  timeout 20s ros2 topic echo /b2/odom \
    nav_msgs/msg/Odometry --field child_frame_id --once 2>/dev/null
)" || fail "B2 odometry did not produce a message within 20 seconds."
[[ "$b2_odom_child_frame_id" == *base_link* ]] ||
  fail "B2 odometry child_frame_id must be base_link: $b2_odom_child_frame_id"

lidar_info=""
for attempt in $(seq 1 10); do
  lidar_info="$(ros2 topic info /lidar_points --verbose 2>/dev/null || true)"
  if [[ "$lidar_info" == *'Type: sensor_msgs/msg/PointCloud2'* ]] &&
     [[ "$lidar_info" =~ Publisher\ count:\ [1-9][0-9]* ]]; then
    break
  fi
  sleep 0.5
done
[[ "$lidar_info" == *'Type: sensor_msgs/msg/PointCloud2'* ]] ||
  fail "/lidar_points is not sensor_msgs/msg/PointCloud2: $lidar_info"
[[ "$lidar_info" =~ Publisher\ count:\ [1-9][0-9]* ]] ||
  fail "/lidar_points has no publisher: $lidar_info"
if [[ "$navigation_mode" == lidar ]]; then
  [[ "$lidar_info" =~ Subscription\ count:\ [1-9][0-9]* ]] ||
    fail "/lidar_points has no Nav2 costmap subscriber: $lidar_info"
fi

lidar_frame_id="$(
  timeout 15s ros2 topic echo /lidar_points sensor_msgs/msg/PointCloud2 \
    --field header.frame_id --once 2>/dev/null
)" || fail "/lidar_points did not produce a PointCloud2 message within 15 seconds."
[[ "$lidar_frame_id" == *hesai_lidar* ]] ||
  fail "/lidar_points frame_id must be hesai_lidar: $lidar_frame_id"

lidar_fields="$(
  timeout 15s ros2 topic echo /lidar_points sensor_msgs/msg/PointCloud2 \
    --field fields --once 2>/dev/null
)" || fail "could not read the /lidar_points PointCloud2 fields within 15 seconds."
for field_name in x y z; do
  # ros2cli prints PointField either as Python repr (name='x') or YAML
  # (name: x), depending on the installed Jazzy CLI version.
  grep -Eq "name[=:][[:space:]]*['\"]?${field_name}['\"]?([,)]|[[:space:]]|$)" <<<"$lidar_fields" ||
    fail "/lidar_points is missing the $field_name field: $lidar_fields"
done

hesai_lidar_tf="$(
  # tf2_echo is continuous. Stop its pipeline after the first complete lookup,
  # while retaining the same 15-second discovery ceiling as the topic checks.
  set +o pipefail
  timeout 15s ros2 run tf2_ros tf2_echo base_link hesai_lidar 2>&1 |
    awk '
      /^At time / { transform_found = 1 }
      transform_found { print }
      transform_found && /^- Rotation: in Quaternion / { exit }
    '
)"
[[ "$hesai_lidar_tf" == *'At time '* ]] &&
  [[ "$hesai_lidar_tf" == *'- Translation: '* ]] &&
  [[ "$hesai_lidar_tf" == *'- Rotation: in Quaternion '* ]] ||
  fail "TF base_link -> hesai_lidar was not available within 15 seconds: $hesai_lidar_tf"

if [[ "$require_wrist_camera" == true ]]; then
  wrist_image_topics=(
    /wrist_camera/color/image_raw
    /wrist_camera/aligned_depth_to_color/image_raw
  )
  for topic_name in "${wrist_image_topics[@]}"; do
    wrist_image_info=""
    for attempt in $(seq 1 20); do
      wrist_image_info="$(ros2 topic info "$topic_name" --verbose 2>/dev/null || true)"
      if [[ "$wrist_image_info" == *'Type: sensor_msgs/msg/Image'* ]] &&
         [[ "$wrist_image_info" =~ Publisher\ count:\ [1-9][0-9]* ]]; then
        break
      fi
      sleep 0.5
    done
    [[ "$wrist_image_info" == *'Type: sensor_msgs/msg/Image'* ]] ||
      fail "$topic_name is not sensor_msgs/msg/Image: $wrist_image_info"
    [[ "$wrist_image_info" =~ Publisher\ count:\ [1-9][0-9]* ]] ||
      fail "$topic_name has no wrist-camera publisher: $wrist_image_info"
  done

  wrist_color_frame_id="$(
    timeout 20s ros2 topic echo /wrist_camera/color/camera_info \
      sensor_msgs/msg/CameraInfo --field header.frame_id --once 2>/dev/null
  )" || fail "wrist color CameraInfo did not produce a message within 20 seconds."
  [[ "$wrist_color_frame_id" == *wrist_camera_color_optical_frame* ]] ||
    fail "wrist color CameraInfo has the wrong frame_id: $wrist_color_frame_id"

  wrist_depth_frame_id="$(
    timeout 20s ros2 topic echo /wrist_camera/aligned_depth_to_color/camera_info \
      sensor_msgs/msg/CameraInfo --field header.frame_id --once 2>/dev/null
  )" || fail "wrist aligned-depth CameraInfo did not produce a message within 20 seconds."
  [[ "$wrist_depth_frame_id" == *wrist_camera_color_optical_frame* ]] ||
    fail "aligned wrist depth must use the color optical frame: $wrist_depth_frame_id"

  wrist_color_encoding="$(
    timeout 20s ros2 topic echo /wrist_camera/color/image_raw \
      sensor_msgs/msg/Image --field encoding --once 2>/dev/null
  )" || fail "wrist color image did not produce a message within 20 seconds."
  [[ "$wrist_color_encoding" == *rgb8* || "$wrist_color_encoding" == *bgr8* ||
     "$wrist_color_encoding" == *rgba8* || "$wrist_color_encoding" == *bgra8* ]] ||
    fail "unsupported wrist color encoding: $wrist_color_encoding"

  wrist_depth_encoding="$(
    timeout 20s ros2 topic echo /wrist_camera/aligned_depth_to_color/image_raw \
      sensor_msgs/msg/Image --field encoding --once 2>/dev/null
  )" || fail "wrist aligned-depth image did not produce a message within 20 seconds."
  if [[ "$wrist_depth_encoding" == *32FC1* ]]; then
    wrist_depth_contract="32FC1 metres (Isaac Sim)"
  elif [[ "$wrist_depth_encoding" == *16UC1* ]]; then
    wrist_depth_contract="16UC1 millimetres / depth_scale 0.001 (RealSense)"
  else
    fail "unsupported wrist aligned-depth encoding: $wrist_depth_encoding"
  fi
fi

map_to_base_tf="$(
  # This lookup traverses the wrapper-owned map -> odom -> zed_camera_link
  # edges and this package's zed_camera_link -> base_link mounting edge.  It
  # therefore catches a dead VIO stream and any accidental TF double-parent.
  set +o pipefail
  timeout 15s ros2 run tf2_ros tf2_echo map base_link 2>&1 |
    awk '
      /^At time / { transform_found = 1 }
      transform_found { print }
      transform_found && /^- Rotation: in Quaternion / { exit }
    '
)"
[[ "$map_to_base_tf" == *'At time '* ]] &&
  [[ "$map_to_base_tf" == *'- Translation: '* ]] &&
  [[ "$map_to_base_tf" == *'- Rotation: in Quaternion '* ]] ||
  fail "TF map -> base_link was not available within 15 seconds: $map_to_base_tf"

if [[ "$navigation_mode" == depth ]]; then
  timeout 15s ros2 topic echo /nvblox_node/static_map_slice --once >/dev/null 2>&1 ||
    fail "Nvblox static_map_slice did not produce a message within 15 seconds."
fi

for lifecycle_node in controller_server planner_server bt_navigator velocity_smoother; do
  lifecycle_state="$(ros2 lifecycle get "/$lifecycle_node" 2>/dev/null)" ||
    fail "could not query lifecycle node /$lifecycle_node."
  [[ "$lifecycle_state" == *'active [3]'* ]] ||
    fail "/$lifecycle_node is not active: $lifecycle_state"
done

# In composed Nav2, the controller and planner create their costmap nodes
# dynamically. If the shared component container was not started with the
# complete Nav2 parameter file, those child nodes silently fall back to the
# default static/obstacle layers even though all lifecycle nodes become active.
for costmap_node in /local_costmap/local_costmap /global_costmap/global_costmap; do
  costmap_plugins="$(ros2 param get "$costmap_node" plugins 2>/dev/null)" ||
    fail "could not read the plugins parameter from $costmap_node."
  if [[ "$navigation_mode" == depth ]]; then
    [[ "$costmap_plugins" == *nvblox_layer* ]] &&
      [[ "$costmap_plugins" == *inflation_layer* ]] ||
      fail "$costmap_node is not using the Nvblox costmap contract: $costmap_plugins"
    [[ "$costmap_plugins" != *static_layer* ]] &&
      [[ "$costmap_plugins" != *obstacle_layer* ]] ||
      fail "$costmap_node fell back to Nav2 default layers: $costmap_plugins"

    costmap_slice_topic="$(
      ros2 param get "$costmap_node" nvblox_layer.nvblox_map_slice_topic 2>/dev/null
    )" || fail "could not read the Nvblox map slice topic from $costmap_node."
    [[ "$costmap_slice_topic" == *'/nvblox_node/static_map_slice'* ]] ||
      fail "$costmap_node uses the wrong Nvblox map slice topic: $costmap_slice_topic"
  else
    [[ "$costmap_plugins" == *obstacle_layer* ]] &&
      [[ "$costmap_plugins" == *inflation_layer* ]] ||
      fail "$costmap_node is not using the XT32 ObstacleLayer contract: $costmap_plugins"
    [[ "$costmap_plugins" != *nvblox_layer* ]] &&
      [[ "$costmap_plugins" != *static_layer* ]] ||
      fail "$costmap_node loaded an unexpected map layer: $costmap_plugins"

    lidar_topic_parameter="$(
      ros2 param get "$costmap_node" obstacle_layer.xt32.topic 2>/dev/null
    )" || fail "could not read the XT32 topic parameter from $costmap_node."
    [[ "$lidar_topic_parameter" == *'/lidar_points'* ]] ||
      fail "$costmap_node uses the wrong XT32 topic: $lidar_topic_parameter"
  fi
done

cmd_vel_info="$(ros2 topic info /cmd_vel --verbose 2>/dev/null)" ||
  fail "could not inspect /cmd_vel publishers."
[[ "$cmd_vel_info" == *'Publisher count: 1'* ]] ||
  fail "/cmd_vel must have exactly one publisher (the policy watchdog): $cmd_vel_info"
[[ "$cmd_vel_info" == *'Node name: cmd_vel_watchdog'* ]] ||
  fail "/cmd_vel is not published exclusively by cmd_vel_watchdog: $cmd_vel_info"

printf 'Isaac ROS runtime check: PASS\n'
printf '  official ZED container: /zed/zed_container\n'
printf '  ZED image transport: %s\n' "$zed_transport"
printf '  B2 odometry: /b2/odom, child base_link, lever-arm corrected\n'
printf '  ZED TF: map -> odom -> zed_camera_link -> base_link\n'
printf '  XT32 TF: base_link -> hesai_lidar\n'
if [[ "$require_wrist_camera" == true ]]; then
  printf '  wrist RGB-D: aligned color optical frame, depth %s\n' "$wrist_depth_contract"
fi
if [[ "$navigation_mode" == depth ]]; then
  printf '  obstacle source: ZED depth -> Isaac ROS Nvblox\n'
  printf '  shared algorithm container: Nvblox + composed Nav2\n'
  printf '  ZED depth/color/pose/odom: publishing and connected to Nvblox\n'
  printf '  Nvblox map slice: receiving data\n'
  printf '  Nav2 lifecycle: active; local/global costmaps use Nvblox layers\n'
  printf '  XT32 PointCloud2: /lidar_points available independent of ZED Nvblox\n'
else
  printf '  obstacle source: XT32 /lidar_points -> Nav2 ObstacleLayer\n'
  printf '  Nav2 execution: standalone official Nav2 processes; Nvblox is not required\n'
  printf '  Nav2 lifecycle: active; local/global costmaps use XT32 ObstacleLayer\n'
fi
printf '  /cmd_vel publisher: cmd_vel_watchdog only\n'
