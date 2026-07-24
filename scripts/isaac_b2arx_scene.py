from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

# Isaac Sim's pip launcher does not put its bundled Jazzy libraries on the
# process-start loader path.  This must run before AppLauncher/Kit imports;
# otherwise enabling isaacsim.ros2.bridge fails on libament_index_cpp.so.
from isaac_ros_env import ensure_ros_bridge_process_environment

ensure_ros_bridge_process_environment()

from isaaclab.app import AppLauncher
from zed_isaac_sim import (
    ZED_CORE_ROOT_REL_PATH,
    ZED_STREAM_FPS,
    ZED_STREAM_PORT,
    ZED_STREAM_RESOLUTION,
    ZED_STREAM_RESOLUTIONS,
    ZED_STREAM_TRANSPORT,
    ZED_X_BASELINE_M,
    ZED_X_CAMERA_PRIMS,
    ZED_X_IMU_PRIM,
    add_zed_startup_kit_args,
    enable_zed_extension,
    install_zed_stream_status_logger,
    remove_zed_stream_status_logger,
    report_zed_stream_status,
    setup_zed_stream_graph,
    stream_dimensions,
    stream_intrinsics,
    validate_zed_configuration,
    validate_zed_installation,
    validate_zed_stream_settings,
)


DEFAULT_ROBOT_USD = Path(__file__).resolve().parents[1] / "assets" / "my_B2Arx" / "my_b2arx" / "my_robot.usd"
ZED_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "outputs" / "zed_x"
DEFAULT_NAV2_DEPLOY_CONFIG = Path(__file__).resolve().parents[1] / "config" / "policies" / "basic_locomotion.yaml"


parser = argparse.ArgumentParser(description="Spawn a B2+ARX R5 manipulation scene in Isaac Lab.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of scene copies.")
parser.add_argument("--env_spacing", type=float, default=4.0, help="Spacing between cloned environments.")
parser.add_argument("--robot_usd", type=str, default=str(DEFAULT_ROBOT_USD), help="Path to the B2+R5 robot USD.")
parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means run until the app closes.")
parser.add_argument("--no_workspace", action="store_true", help="Do not spawn the table and manipulation objects.")
parser.add_argument(
    "--no_zed", "--no_zed2i", dest="no_zed", action="store_true",
    help=(
        "Disable the embedded ZED X data path and extension; the sensor model remains part of the core robot USD "
        "(--no_zed2i is a legacy alias)."
    ),
)
parser.add_argument(
    "--no_zed_sdk_stream",
    action="store_true",
    help="Keep the embedded ZED X active for local probes without starting the ZED SDK simulation stream.",
)
parser.add_argument(
    "--save_zed_frames",
    action="store_true",
    help="Save renderer-side ZED X left/right RGB under outputs/zed_x (not ZED SDK depth).",
)
parser.add_argument(
    "--zed_stream_resolution", choices=tuple(ZED_STREAM_RESOLUTIONS), default=ZED_STREAM_RESOLUTION,
    help="Official Stereolabs stream resolution passed to the ZED Camera Helper.",
)
parser.add_argument(
    "--zed_stream_fps", type=int, choices=(15, 30, 60, 120), default=ZED_STREAM_FPS,
    help="Official Stereolabs stream frame rate; HD1200 accepts 15/30/60 Hz.",
)
parser.add_argument(
    "--zed_stream_port", type=int, default=ZED_STREAM_PORT,
    help="Even port consumed by zed_wrapper sim_port (default: 30000).",
)
parser.add_argument(
    "--zed_stream_transport", choices=("BOTH", "NETWORK", "IPC"), default=ZED_STREAM_TRANSPORT,
    help="Official Stereolabs stream transport. NETWORK supports local or remote receivers; "
    "set the receiver-side sim_address in the ROS launch. IPC is Linux-only.",
)
parser.add_argument(
    "--control_mode",
    choices=("hold", "policy"),
    default="hold",
    help="hold keeps the current fixed-pose demo; policy runs the sim2sim2real deployment FSM and ONNX policy.",
)
DEFAULT_DEPLOY_CONFIG = Path(__file__).resolve().parent / "policy_deploy" / "deploy_config.example.yaml"
parser.add_argument(
    "--deploy_config",
    type=str,
    default=str(DEFAULT_DEPLOY_CONFIG),
    help="deploy_config.yaml for --control_mode policy (policy/scene/deploy/input).",
)
parser.add_argument("--print_policy_debug", action="store_true", help="Print policy FSM/action/target diagnostics.")
parser.add_argument(
    "--scene_asset",
    choices=("default", "minimal", "grid", "rough_plane", "warehouse", "warehouse_local", "hospital"),
    default="default",
    help="Background USD. 'default' = local Simple_Warehouse; 'minimal' = bare ground plane; "
    "others are official Nucleus assets. deploy_config scene.environment_usd overrides 'default'.",
)
parser.add_argument(
    "--environment_usd",
    type=str,
    default="",
    help="Custom environment/background USD path or URL. Overrides --scene_asset when set.",
)
parser.add_argument(
    "--arm_gain_profile",
    choices=("identified", "train"),
    default="identified",
    help="Arm PD gains. 'identified' uses the 2026-06-05 pure-PD fit; 'train' uses the nominal Isaac training contract.",
)
parser.add_argument(
    "--viewer_camera",
    choices=("scene", "zed", "zed_left", "zed_right", "wrist"),
    default="scene",
    help="Select the viewport camera; zed is an alias for zed_left and wrist uses the embedded RealSense color camera.",
)
parser.add_argument(
    "--target_object",
    choices=("red_box", "blue_cylinder", "green_ball"),
    default="red_box",
    help="Workspace object used for EE sphere target debug output.",
)
parser.add_argument(
    "--print_ee_target_debug",
    action="store_true",
    help="Print selected object world pose converted to deploy-side EE sphere target.",
)
parser.add_argument(
    "--print_zed_debug", "--print_zed2i_debug", dest="print_zed_debug",
    action="store_true",
    help="Print official ZED X extension, stream settings and live camera poses.",
)
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric API and use USD instead.")
parser.add_argument("--ros2", action="store_true",
                    help="启用 ROS2 OmniGraph /clock 和内嵌 XT32 点云；ZED ROS 话题由官方 zed_wrapper 发布。")
parser.add_argument(
    "--no_xt32_pointcloud",
    action="store_true",
    help="Do not publish the embedded Hesai XT32 PointCloud2 when --ros2 is active.",
)
parser.add_argument(
    "--xt32_pointcloud_topic",
    type=str,
    default="/lidar_points",
    help="ROS2 PointCloud2 topic for the embedded XT32; defaults to the real Hesai driver contract.",
)
parser.add_argument(
    "--xt32_frame_id",
    type=str,
    default="hesai_lidar",
    help="ROS frame_id stamped on the embedded XT32 point cloud.",
)
parser.add_argument(
    "--show_xt32_debug_view",
    action="store_true",
    help="Draw the RTX XT32 point cloud inside Isaac Sim while publishing it.",
)
parser.add_argument(
    "--wrist_camera",
    action="store_true",
    help=(
        "Publish the embedded official RealSense color camera as aligned RGB-D "
        "through Isaac Sim ROS2 Camera Helpers."
    ),
)
parser.add_argument(
    "--wrist_camera_prim_path",
    type=str,
    default="auto",
    help=(
        "Color-camera prim inside the instantiated robot. Use 'auto', an "
        "absolute stage path, or a path relative to the robot prim."
    ),
)
parser.add_argument("--wrist_camera_width", type=int, default=1280)
parser.add_argument("--wrist_camera_height", type=int, default=720)
parser.add_argument("--wrist_camera_frame_skip_count", type=int, default=0)
parser.add_argument(
    "--wrist_camera_frame_id",
    type=str,
    default="wrist_camera_color_optical_frame",
)
parser.add_argument(
    "--wrist_camera_color_topic",
    type=str,
    default="/wrist_camera/color/image_raw",
)
parser.add_argument(
    "--wrist_camera_color_info_topic",
    type=str,
    default="/wrist_camera/color/camera_info",
)
parser.add_argument(
    "--wrist_camera_aligned_depth_topic",
    type=str,
    default="/wrist_camera/aligned_depth_to_color/image_raw",
)
parser.add_argument(
    "--wrist_camera_aligned_depth_info_topic",
    type=str,
    default="/wrist_camera/aligned_depth_to_color/camera_info",
)
parser.add_argument(
    "--nav2",
    action="store_true",
    help="Enable the Nav2 simulation contract: ROS 2 clock, /cmd_vel subscription, and B2 locomotion policy.",
)
parser.add_argument("--ros2_domain_id", type=int, default=23,
                    help="ROS_DOMAIN_ID, 必须与 Thor 端一致 (spec §4)。")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.nav2:
    if args_cli.no_zed or args_cli.no_zed_sdk_stream:
        raise ValueError("--nav2 requires the official ZED X asset and ZED SDK simulation stream")
    if args_cli.no_xt32_pointcloud:
        raise ValueError("--nav2 requires the embedded XT32 PointCloud2 data path")
    args_cli.ros2 = True
    args_cli.control_mode = "policy"
    if Path(args_cli.deploy_config).resolve() == DEFAULT_DEPLOY_CONFIG.resolve():
        args_cli.deploy_config = str(DEFAULT_NAV2_DEPLOY_CONFIG)

validate_zed_configuration()
validate_zed_stream_settings(
    args_cli.zed_stream_resolution,
    args_cli.zed_stream_fps,
    args_cli.zed_stream_port,
    args_cli.zed_stream_transport,
)
if not args_cli.no_zed and not args_cli.no_zed_sdk_stream and args_cli.num_envs != 1:
    raise ValueError(
        "The official ZED SDK stream currently supports --num_envs 1; "
        "each additional camera requires its own helper graph and even port"
    )
if args_cli.ros2 and not args_cli.no_xt32_pointcloud and args_cli.num_envs != 1:
    raise ValueError("The embedded XT32 ROS2 point cloud currently supports --num_envs 1")
if args_cli.wrist_camera:
    if not args_cli.ros2:
        raise ValueError("--wrist_camera requires --ros2")
    if args_cli.num_envs != 1:
        raise ValueError("The wrist RGB-D ROS2 graph currently supports --num_envs 1")
    if args_cli.wrist_camera_width <= 0 or args_cli.wrist_camera_height <= 0:
        raise ValueError("Wrist camera width and height must be positive")
    if args_cli.wrist_camera_frame_skip_count < 0:
        raise ValueError("Wrist camera frame skip count must be non-negative")
if not args_cli.no_zed:
    validate_zed_installation()
    # Isaac physics sensors are instantiated by a manager initialized during
    # Kit startup.  Dynamically enabling that extension after AppLauncher has
    # created SimulationApp leaves even the official ZED IMU permanently
    # invalid and prevents the native Stereolabs stream node from executing.
    args_cli.kit_args = add_zed_startup_kit_args(args_cli.kit_args)
if args_cli.ros2 and not args_cli.no_xt32_pointcloud:
    # Isaac Sim 5.1 no longer enables Motion BVH by default. RTX lidar on a
    # moving robot needs these settings before SimulationApp starts or the
    # accumulated full scan omits motion effects.
    motion_bvh_args = (
        "--/renderer/raytracingMotion/enabled=true",
        "--/renderer/raytracingMotion/enableHydraEngineMasking=true",
        "--/renderer/raytracingMotion/enabledForHydraEngines='0'",
    )
    kit_tokens = args_cli.kit_args.split()
    for motion_arg in motion_bvh_args:
        setting_name = motion_arg.split("=", 1)[0]
        if not any(token == setting_name or token.startswith(f"{setting_name}=") for token in kit_tokens):
            kit_tokens.append(motion_arg)
    args_cli.kit_args = " ".join(kit_tokens)
if (
    not args_cli.no_zed
    or (args_cli.ros2 and not args_cli.no_xt32_pointcloud)
    or args_cli.wrist_camera
):
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
zed_stream_logger_state = None

if not args_cli.no_zed and not args_cli.no_zed_sdk_stream:
    zed_extension_id = enable_zed_extension()
    zed_stream_logger_state = install_zed_stream_status_logger()
    print(
        f"[INFO]: Enabled official Stereolabs Isaac Sim extension {zed_extension_id}",
        flush=True,
    )

# Enable the ROS2 bridge extension right after the app launches, before the physics
# device/backend is negotiated (SimulationContext). Enabling it later — between
# SimulationContext and InteractiveScene — leaves scene.env_origins on CPU while the
# robot lands on cuda:0, crashing reset_scene with a device mismatch. Official IsaacLab
# scripts (e.g. benchmarks/benchmark_non_rl.py) enable extensions at this same point.
if args_cli.ros2:
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    if not args_cli.no_xt32_pointcloud:
        enable_extension("isaacsim.sensors.rtx")

import matplotlib.pyplot as plt
import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sim import SimulationContext
from isaaclab.sim.schemas import ArticulationRootPropertiesCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from pxr import Usd, UsdPhysics
from ee_sphere import target_world_to_sphere
from policy_deploy.command_sources import make_command_source
from policy_deploy.deploy_config import load_deploy_config, verify_policy_bundle
from policy_deploy.isaac_controller import B2ArxIsaacPolicyController
B2_JOINT_POS = {
    "b2_description_FL_hip_joint": 0.15,
    "b2_description_FL_thigh_joint": 0.67,
    "b2_description_FL_calf_joint": -1.32,
    "b2_description_FR_hip_joint": -0.15,
    "b2_description_FR_thigh_joint": 0.67,
    "b2_description_FR_calf_joint": -1.32,
    "b2_description_RL_hip_joint": 0.15,
    "b2_description_RL_thigh_joint": 0.67,
    "b2_description_RL_calf_joint": -1.32,
    "b2_description_RR_hip_joint": -0.15,
    "b2_description_RR_thigh_joint": 0.67,
    "b2_description_RR_calf_joint": -1.32,
}


R5_HOME_POS = {
    "R5a_joint1": 0.0,
    "R5a_joint2": 0.0,
    "R5a_joint3": 0.0,
    "R5a_joint4": 0.0,
    "R5a_joint5": 0.0,
    "R5a_joint6": 0.0,
    "R5a_joint7": 0.0,
    "R5a_joint8": 0.0,
}

# Sensor paths inside the user-authored core robot USD.  The scene never spawns
# duplicate sensor assets; the official ZED helper and RTX Lidar graph bind to
# these composed prims after the robot reference is instantiated.
ROBOT_PRIM_PATH = "/World/envs/env_0/Robot"
ZED_ASSET_PRIM_PATH = f"{ROBOT_PRIM_PATH}/{ZED_CORE_ROOT_REL_PATH}"
ZED_LEFT_CAMERA_PRIM_PATH = f"{ZED_ASSET_PRIM_PATH}/{ZED_X_CAMERA_PRIMS['left']}"
ZED_RIGHT_CAMERA_PRIM_PATH = f"{ZED_ASSET_PRIM_PATH}/{ZED_X_CAMERA_PRIMS['right']}"
ZED_IMU_PRIM_PATH = f"{ZED_ASSET_PRIM_PATH}/{ZED_X_IMU_PRIM}"
XT32_LIDAR_REL_PATH = "b2_description/XT_32/PandarXT_32_10hz"
XT32_LIDAR_PRIM_PATH = f"{ROBOT_PRIM_PATH}/{XT32_LIDAR_REL_PATH}"
# The current manually authored core USD uses the first layout.  The second is
# retained for URDF-converter variants that nest all links below the base prim.
WRIST_COLOR_CAMERA_REL_PATHS = (
    "R5a_link6/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
    "b2_description/R5a_link6/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
)
ZED_CAMERA_PRIM_PATHS = {
    "zed": ZED_LEFT_CAMERA_PRIM_PATH,
    "zed_left": ZED_LEFT_CAMERA_PRIM_PATH,
    "zed_right": ZED_RIGHT_CAMERA_PRIM_PATH,
}
ZED_IMAGE_WIDTH, ZED_IMAGE_HEIGHT = stream_dimensions(args_cli.zed_stream_resolution)
ZED_FX, ZED_FY, ZED_CX, ZED_CY = stream_intrinsics(args_cli.zed_stream_resolution)

LEG_HIP_JOINTS = [
    "b2_description_FL_hip_joint",
    "b2_description_FR_hip_joint",
    "b2_description_RL_hip_joint",
    "b2_description_RR_hip_joint",
]
LEG_THIGH_JOINTS = [
    "b2_description_FL_thigh_joint",
    "b2_description_FR_thigh_joint",
    "b2_description_RL_thigh_joint",
    "b2_description_RR_thigh_joint",
]
LEG_CALF_JOINTS = [
    "b2_description_FL_calf_joint",
    "b2_description_FR_calf_joint",
    "b2_description_RL_calf_joint",
    "b2_description_RR_calf_joint",
]

ARM_JOINTS = ["R5a_joint1", "R5a_joint2", "R5a_joint3", "R5a_joint4", "R5a_joint5", "R5a_joint6"]


LOCAL_WAREHOUSE_USD = os.environ.get(
    "B2ARX_WAREHOUSE_USD",
    str(
        Path.home()
        / "Documents/isaac-sim-assets-environments-5.1.0"
        / "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    ),
)

OFFICIAL_SCENE_ASSETS = {
    "grid": f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd",
    "rough_plane": f"{ISAAC_NUCLEUS_DIR}/Environments/Terrains/rough_plane.usd",
    "warehouse": f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd",
    "warehouse_local": LOCAL_WAREHOUSE_USD,
    "hospital": f"{ISAAC_NUCLEUS_DIR}/Environments/Hospital/hospital.usd",
}

def _resolve_usd_path(usd_path: str) -> str:
    """Resolve local USD paths while leaving Isaac/Nucleus URLs untouched."""
    if usd_path.startswith(("http://", "https://", "omniverse://")):
        return usd_path
    return str(Path(usd_path).expanduser().resolve())


# Load the deploy config once at module scope (policy mode only) so the scene cfg,
# built below, can honor scene.environment_usd. make_policy_controller reuses this.
DEPLOY_CFG = load_deploy_config(args_cli.deploy_config) if args_cli.control_mode == "policy" else None

# A policy-specific robot USD is part of the dynamics contract. Respect it
# when the caller has not explicitly overridden --robot_usd on the CLI.
if (
    DEPLOY_CFG is not None
    and DEPLOY_CFG.scene.robot_usd is not None
    and Path(args_cli.robot_usd).expanduser().resolve() == DEFAULT_ROBOT_USD.resolve()
):
    args_cli.robot_usd = str(DEPLOY_CFG.scene.robot_usd)


def _config_environment_usd() -> str | None:
    """environment_usd from deploy_config.scene, already path/URL resolved by the loader."""
    if DEPLOY_CFG is None or DEPLOY_CFG.scene.environment_usd is None:
        return None
    return str(DEPLOY_CFG.scene.environment_usd)


def selected_environment_usd() -> str | None:
    """Priority: CLI --environment_usd > config scene > CLI --scene_asset > default local warehouse.

    --scene_asset minimal forces the bare ground plane (no background), overriding the
    warehouse default; use it when isolating robot stability.
    """
    if args_cli.environment_usd:
        return _resolve_usd_path(args_cli.environment_usd)
    if args_cli.scene_asset == "minimal":
        return None
    cfg_env = _config_environment_usd()
    if cfg_env is not None:
        return cfg_env
    if args_cli.scene_asset == "default":
        return _resolve_usd_path(LOCAL_WAREHOUSE_USD)
    return OFFICIAL_SCENE_ASSETS[args_cli.scene_asset]


SELECTED_ENVIRONMENT_USD = selected_environment_usd()


def make_robot_cfg(robot_usd: str) -> ArticulationCfg:
    if args_cli.arm_gain_profile == "identified":
        # Source: the actuator-identification fit documented in README.md.
        # Pure PD fit. delay_steps are recorded in that file but IdealPDActuatorCfg only supports kp/kd/limits.
        arm_actuators = {
            "arm_joint1": IdealPDActuatorCfg(
                joint_names_expr=["R5a_joint1"],
                effort_limit=64.0,
                effort_limit_sim=64.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=16.002533696088847,
                damping=0.9653550775562955,
            ),
            "arm_joint2": IdealPDActuatorCfg(
                joint_names_expr=["R5a_joint2"],
                effort_limit=64.0,
                effort_limit_sim=64.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=26.66343352125888,
                damping=1.898936300858176,
            ),
            "arm_joint3": IdealPDActuatorCfg(
                joint_names_expr=["R5a_joint3"],
                effort_limit=64.0,
                effort_limit_sim=64.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=31.72318363299385,
                damping=2.4018660298707024,
            ),
            "arm_joint4": IdealPDActuatorCfg(
                joint_names_expr=["R5a_joint4"],
                effort_limit=64.0,
                effort_limit_sim=64.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=5.3742454192848506,
                damping=0.18449434631628026,
            ),
            "arm_joint5": IdealPDActuatorCfg(
                joint_names_expr=["R5a_joint5"],
                effort_limit=64.0,
                effort_limit_sim=64.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=3.088030381518184,
                damping=0.14579060361110824,
            ),
            "arm_joint6": IdealPDActuatorCfg(
                joint_names_expr=["R5a_joint6"],
                effort_limit=64.0,
                effort_limit_sim=64.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=4.858456866357783,
                damping=0.04160561372766666,
            ),
        }
    else:
        arm_actuators = {
            "arm_shoulder": IdealPDActuatorCfg(
                joint_names_expr=["R5a_joint2"],
                effort_limit=64.0,
                effort_limit_sim=64.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=128.0,
                damping=3.0,
            ),
            "arm_other": IdealPDActuatorCfg(
                joint_names_expr=["R5a_joint1", "R5a_joint3", "R5a_joint4", "R5a_joint5", "R5a_joint6"],
                effort_limit=64.0,
                effort_limit_sim=64.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=64.0,
                damping=1.5,
            ),
        }

    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=robot_usd,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=ArticulationRootPropertiesCfg(
                # The model_29999 training environment explicitly disabled
                # articulation self-collisions; keep deployment identical.
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.55),
            joint_pos={**B2_JOINT_POS, **R5_HOME_POS},
        ),
        actuators={
            "leg_hip_joints": IdealPDActuatorCfg(
                joint_names_expr=LEG_HIP_JOINTS,
                effort_limit=200.0,
                effort_limit_sim=200.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=300.0,
                damping=7.5,
            ),
            "leg_thigh_joints": IdealPDActuatorCfg(
                joint_names_expr=LEG_THIGH_JOINTS,
                effort_limit=200.0,
                effort_limit_sim=200.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=300.0,
                damping=7.5,
            ),
            "leg_calf_joints": IdealPDActuatorCfg(
                joint_names_expr=LEG_CALF_JOINTS,
                effort_limit=320.0,
                effort_limit_sim=320.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=500.0,
                damping=12.5,
            ),
            **arm_actuators,
            "gripper": IdealPDActuatorCfg(
                joint_names_expr=["R5a_joint[7-8]"],
                effort_limit=64.0,
                effort_limit_sim=64.0,
                velocity_limit=100.0,
                velocity_limit_sim=100.0,
                stiffness=64.0,
                damping=1.5,
            ),
        },
    )


@configclass
class B2ArxManipulationSceneCfg(InteractiveSceneCfg):
    """B2+ARX R5 scene with a simple manipulation workspace."""

    ground = None if SELECTED_ENVIRONMENT_USD else AssetBaseCfg(
        prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg()
    )

    environment = None if SELECTED_ENVIRONMENT_USD is None else AssetBaseCfg(
        prim_path="/World/Environment",
        spawn=sim_utils.UsdFileCfg(usd_path=SELECTED_ENVIRONMENT_USD),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.78, 0.78, 0.74)),
    )

    robot: ArticulationCfg = make_robot_cfg(str(Path(args_cli.robot_usd).expanduser().resolve())).replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )

    work_table = None if args_cli.no_workspace else AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/WorkTable",
        spawn=sim_utils.CuboidCfg(
            size=(0.82, 0.58, 0.06),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.46, 0.48, 0.45), roughness=0.72),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(1.28, 0.0, 0.42)),
    )

    red_box = None if args_cli.no_workspace else RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/red_box",
        spawn=sim_utils.CuboidCfg(
            size=(0.085, 0.085, 0.065),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=8, solver_velocity_iteration_count=0
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.12),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.86, 0.08, 0.05), roughness=0.58),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(1.16, -0.14, 0.485)),
    )

    blue_cylinder = None if args_cli.no_workspace else RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/blue_cylinder",
        spawn=sim_utils.CylinderCfg(
            radius=0.045,
            height=0.11,
            axis="Z",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=8, solver_velocity_iteration_count=0
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.10),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.08, 0.24, 0.86), roughness=0.48),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(1.30, 0.02, 0.51)),
    )

    green_ball = None if args_cli.no_workspace else RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/green_ball",
        spawn=sim_utils.SphereCfg(
            radius=0.05,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=8, solver_velocity_iteration_count=0
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.08),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.05, 0.62, 0.20), roughness=0.52),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(1.43, -0.10, 0.50)),
    )

    # Optional renderer-side probes are created only for --save_zed_frames.
    # They bind to the cameras already embedded in the core robot USD. They are
    # not the Nvblox input: zed_wrapper still publishes SDK-computed depth.
    zed_left_camera = None if args_cli.no_zed or not args_cli.save_zed_frames else CameraCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{ZED_CORE_ROOT_REL_PATH}/{ZED_X_CAMERA_PRIMS['left']}",
        update_period=1.0 / float(args_cli.zed_stream_fps),
        height=ZED_IMAGE_HEIGHT,
        width=ZED_IMAGE_WIDTH,
        data_types=["rgb"],
        spawn=None,
    )

    zed_right_camera = None if args_cli.no_zed or not args_cli.save_zed_frames else CameraCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{ZED_CORE_ROOT_REL_PATH}/{ZED_X_CAMERA_PRIMS['right']}",
        update_period=1.0 / float(args_cli.zed_stream_fps),
        height=ZED_IMAGE_HEIGHT,
        width=ZED_IMAGE_WIDTH,
        data_types=["rgb"],
        spawn=None,
    )


def wrist_color_camera_candidate_paths() -> tuple[str, ...]:
    requested = args_cli.wrist_camera_prim_path.strip()
    if requested == "auto":
        return tuple(f"{ROBOT_PRIM_PATH}/{path}" for path in WRIST_COLOR_CAMERA_REL_PATHS)
    if requested.startswith("/"):
        return (requested,)
    return (f"{ROBOT_PRIM_PATH}/{requested.strip('/')}",)


def resolve_wrist_color_camera_prim_path() -> str:
    """Resolve and type-check the existing official RealSense color camera."""
    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    candidates = wrist_color_camera_candidate_paths()
    for prim_path in candidates:
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid() and prim.IsA(UsdGeom.Camera):
            return prim_path
    raise RuntimeError(
        "The selected core robot USD has no UsdGeom.Camera at the requested "
        "wrist color-camera path(s): " + ", ".join(candidates)
    )


def validate_core_sensor_prims(
    *, require_zed: bool, require_xt32: bool, require_wrist_camera: bool = False
) -> str | None:
    """Fail early when an alternate robot USD does not satisfy the sensor contract."""
    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    missing: list[str] = []

    if require_zed:
        for prim_path in (
            ZED_ASSET_PRIM_PATH,
            ZED_LEFT_CAMERA_PRIM_PATH,
            ZED_RIGHT_CAMERA_PRIM_PATH,
            ZED_IMU_PRIM_PATH,
        ):
            if not stage.GetPrimAtPath(prim_path).IsValid():
                missing.append(prim_path)
        if not missing:
            for camera_path in (ZED_LEFT_CAMERA_PRIM_PATH, ZED_RIGHT_CAMERA_PRIM_PATH):
                if not stage.GetPrimAtPath(camera_path).IsA(UsdGeom.Camera):
                    raise RuntimeError(f"Embedded ZED camera prim has the wrong type: {camera_path}")
            if stage.GetPrimAtPath(ZED_IMU_PRIM_PATH).GetTypeName() != "IsaacImuSensor":
                raise RuntimeError(f"Embedded ZED IMU prim has the wrong type: {ZED_IMU_PRIM_PATH}")

            zed_root = stage.GetPrimAtPath(ZED_ASSET_PRIM_PATH)
            rigid_enabled = zed_root.GetAttribute("physics:rigidBodyEnabled")
            if zed_root.HasAPI(UsdPhysics.RigidBodyAPI) or not rigid_enabled or rigid_enabled.Get() is not False:
                raise RuntimeError(
                    "Embedded ZED root must remain a non-rigid compound payload with "
                    f"physics:rigidBodyEnabled=false: {ZED_ASSET_PRIM_PATH}"
                )

    if require_xt32:
        lidar_prim = stage.GetPrimAtPath(XT32_LIDAR_PRIM_PATH)
        if not lidar_prim.IsValid():
            missing.append(XT32_LIDAR_PRIM_PATH)
        else:
            if lidar_prim.GetTypeName() != "OmniLidar" or not lidar_prim.HasAPI(
                "OmniSensorGenericLidarCoreAPI"
            ):
                raise RuntimeError(f"Embedded XT32 prim is not an official OmniLidar: {XT32_LIDAR_PRIM_PATH}")
            output_frame = lidar_prim.GetAttribute("omni:sensor:Core:outputFrameOfReference").Get()
            if output_frame != "SENSOR":
                raise RuntimeError(
                    "Embedded XT32 point coordinates must be expressed in its SENSOR frame; "
                    f"got {output_frame!r} at {XT32_LIDAR_PRIM_PATH}"
                )

    wrist_camera_prim_path = None
    if require_wrist_camera:
        wrist_camera_prim_path = resolve_wrist_color_camera_prim_path()

    if missing:
        raise RuntimeError(
            "The selected core robot USD is missing required embedded sensor prims (or their official "
            "payloads did not resolve): " + ", ".join(missing)
        )
    return wrist_camera_prim_path

def reset_scene(scene: InteractiveScene) -> None:
    robot = scene["robot"]
    root_state = robot.data.default_root_state.clone()
    root_state[:, :3] += scene.env_origins
    robot.write_root_pose_to_sim(root_state[:, :7])
    robot.write_root_velocity_to_sim(root_state[:, 7:])

    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    robot.set_joint_position_target(joint_pos)
    robot.set_joint_velocity_target(joint_vel)
    robot.set_joint_effort_target(joint_pos.new_zeros(joint_pos.shape))

    if args_cli.no_workspace:
        scene.reset()
        return

    for object_name in ("red_box", "blue_cylinder", "green_ball"):
        obj = scene[object_name]
        object_state = obj.data.default_root_state.clone()
        object_state[:, :3] += scene.env_origins
        obj.write_root_pose_to_sim(object_state[:, :7])
        obj.write_root_velocity_to_sim(object_state[:, 7:])

    scene.reset()


def print_environment_physics_debug() -> None:
    """Print the loaded environment's physics materials (friction/restitution) and gravity.

    The warehouse floor uses its own USD PhysicsMaterial, not IsaacLab's default ground
    plane. A low ground friction here is a prime suspect for FixStand/ArmPreAlign tilt.
    """
    if SELECTED_ENVIRONMENT_USD is None:
        return
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    env_root = stage.GetPrimAtPath("/World/Environment")
    if not env_root.IsValid():
        return
    found = 0
    for prim in Usd.PrimRange(env_root):
        if prim.HasAPI(UsdPhysics.MaterialAPI) or prim.GetTypeName() == "PhysicsMaterial":
            mat = UsdPhysics.MaterialAPI(prim)
            sf = mat.GetStaticFrictionAttr().Get()
            df = mat.GetDynamicFrictionAttr().Get()
            rest = mat.GetRestitutionAttr().Get()
            print(
                f"[PHYS]: env material {prim.GetPath()} static_friction={sf} "
                f"dynamic_friction={df} restitution={rest}",
                flush=True,
            )
            found += 1
            if found >= 8:
                break
    if found == 0:
        print("[PHYS]: no explicit PhysicsMaterial under /World/Environment "
              "(floor uses PhysX defaults: friction~0.5)", flush=True)


def set_viewport_camera() -> None:
    if args_cli.headless:
        return
    if args_cli.viewer_camera == "scene":
        return
    if args_cli.viewer_camera == "wrist":
        try:
            camera_path = resolve_wrist_color_camera_prim_path()
        except RuntimeError as exc:
            print(f"[WARN]: Could not select wrist camera: {exc}", flush=True)
            return
    elif args_cli.no_zed:
        print(f"[WARN]: --viewer_camera {args_cli.viewer_camera} was requested, but ZED X is disabled.")
        return
    else:
        camera_path = ZED_CAMERA_PRIM_PATHS[args_cli.viewer_camera]
    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        viewport.set_active_camera(camera_path)
        print(f"[INFO]: Viewport switched to {camera_path}", flush=True)
    except Exception as exc:
        print(f"[WARN]: Could not switch viewport to {args_cli.viewer_camera} camera: {exc}", flush=True)


def _image_rgb_uint8(camera) -> np.ndarray | None:
    if "rgb" not in camera.data.output:
        return None
    rgb = camera.data.output["rgb"][0, ..., :3].detach().cpu().numpy()
    return np.clip(rgb, 0, 255).astype(np.uint8)


def save_zed_frame(scene: InteractiveScene, count: int) -> None:
    """Save renderer-side stereo RGB for visual checks, never as SDK depth."""
    if args_cli.no_zed or not args_cli.save_zed_frames:
        return

    ZED_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    saved = []

    for side in ("left", "right"):
        rgb = _image_rgb_uint8(scene[f"zed_{side}_camera"])
        if rgb is None:
            continue
        path = ZED_OUTPUT_DIR / f"{side}_{count:06d}.png"
        plt.imsave(path, rgb)
        saved.append(path.name)

    if saved:
        print(f"[INFO]: Saved renderer-side ZED X RGB to {ZED_OUTPUT_DIR}: {', '.join(saved)}", flush=True)


def _root_yaw_from_quat_wxyz(quat: torch.Tensor) -> float:
    w, x, y, z = [float(v) for v in quat]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def print_ee_target_debug(scene: InteractiveScene, elapsed: float) -> None:
    if args_cli.no_workspace or not args_cli.print_ee_target_debug:
        return

    robot = scene["robot"]
    obj = scene[args_cli.target_object]
    root_pos = robot.data.root_pos_w[0].detach().cpu().numpy()
    root_yaw = _root_yaw_from_quat_wxyz(robot.data.root_quat_w[0].detach().cpu())
    target_world = obj.data.root_pos_w[0].detach().cpu().numpy()
    debug = target_world_to_sphere(target_world, root_pos, root_yaw)
    print(
        "[TARGET]: "
        f"t={elapsed:.2f}s object={args_cli.target_object} "
        f"world={np.array2string(debug.target_world, precision=3, suppress_small=True)} "
        f"center={np.array2string(debug.sphere_center_world, precision=3, suppress_small=True)} "
        f"local_yaw={np.array2string(debug.local_yaw_frame, precision=3, suppress_small=True)} "
        f"sphere_raw={np.array2string(debug.sphere_raw, precision=3, suppress_small=True)} "
        f"sphere_clamped={np.array2string(debug.sphere_clamped, precision=3, suppress_small=True)}",
        flush=True,
    )


def print_zed_debug() -> None:
    if args_cli.no_zed or not args_cli.print_zed_debug:
        return

    print(
        "[ZED_X]: official_source=stereolabs/zed-isaac-sim@v4.3.0 "
        f"core_usd={Path(args_cli.robot_usd).expanduser().resolve()} "
        f"embedded_root={ZED_ASSET_PRIM_PATH} "
        "mount=core_usd_compound parent=Robot/b2_description "
        f"baseline={ZED_X_BASELINE_M:.3f}m",
        flush=True,
    )
    print(
        "[ZED_X]: "
        f"stream={args_cli.zed_stream_resolution}@{args_cli.zed_stream_fps}Hz "
        f"K=[fx={ZED_FX:.3f}, fy={ZED_FY:.3f}, cx={ZED_CX:.3f}, cy={ZED_CY:.3f}] "
        f"port={args_cli.zed_stream_port} transport={args_cli.zed_stream_transport} "
        f"sdk_stream_enabled={not args_cli.no_zed_sdk_stream}",
        flush=True,
    )
    for side, camera_path in (
        ("left", ZED_LEFT_CAMERA_PRIM_PATH),
        ("right", ZED_RIGHT_CAMERA_PRIM_PATH),
    ):
        print(
            f"[ZED_X]: {side:5s} camera_prim={camera_path}",
            flush=True,
        )


def print_zed_camera_world_poses() -> None:
    """Verify official asset mount, optical centers, baseline and +X view direction."""
    if args_cli.no_zed or not args_cli.print_zed_debug:
        return
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    r5a_path = "/World/envs/env_0/Robot/b2_description/R5a"
    r5a_prim = stage.GetPrimAtPath(r5a_path)
    if not r5a_prim.IsValid():
        print(f"[ZED_X_POSE]: R5a not found at {r5a_path}", flush=True)
        return

    r5a_w = UsdGeom.Xformable(r5a_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    r5a_w_inv = r5a_w.GetInverse()
    measured: dict[str, np.ndarray] = {}
    for side, camera_path in (
        ("left", ZED_LEFT_CAMERA_PRIM_PATH),
        ("right", ZED_RIGHT_CAMERA_PRIM_PATH),
    ):
        camera_prim = stage.GetPrimAtPath(camera_path)
        if not camera_prim.IsValid():
            print(f"[ZED_X_POSE]: {side} camera NOT FOUND at {camera_path}", flush=True)
            continue
        camera_w = UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        camera_in_r5a = camera_w * r5a_w_inv
        translation = camera_in_r5a.ExtractTranslation()
        forward_r5a = camera_in_r5a.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0)).GetNormalized()
        measured[side] = np.array(translation, dtype=np.float64)
        print(
            f"[ZED_X_POSE]: {side:5s} pos_r5a="
            f"{np.array2string(measured[side], precision=5, suppress_small=True)} "
            f"forward_r5a={tuple(round(float(value), 5) for value in forward_r5a)}",
            flush=True,
        )

    if "left" in measured and "right" in measured:
        baseline = float(np.linalg.norm(measured["left"] - measured["right"]))
        center = 0.5 * (measured["left"] + measured["right"])
        print(
            "[ZED_X_POSE]: "
            f"measured_center_r5a={np.array2string(center, precision=5, suppress_small=True)} "
            f"measured_baseline={baseline:.5f}m",
            flush=True,
        )

    # The official Stereolabs stream graph is gated by IsaacReadIMU.  Show the
    # composed physics ancestry so a broken mount cannot be mistaken for a
    # working SDK stream merely because its RGB annotators were constructed.
    prim = stage.GetPrimAtPath(ZED_IMU_PRIM_PATH)
    while prim.IsValid() and prim.GetPath() != Usd.Prim().GetPath():
        rigid_body = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        rigid_enabled = (
            UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
            if rigid_body
            else None
        )
        print(
            f"[ZED_X_PHYS]: path={prim.GetPath()} type={prim.GetTypeName()} "
            f"rigid_body={rigid_body} rigid_enabled={rigid_enabled} "
            f"schemas={prim.GetAppliedSchemas()}",
            flush=True,
        )
        prim = prim.GetParent()


def print_zed_imu_reading(step: int) -> None:
    """Read the untouched official IMU through Isaac Sim's sensor interface."""
    if args_cli.no_zed or not args_cli.print_zed_debug:
        return
    from isaacsim.sensors.physics import _sensor

    reading = _sensor.acquire_imu_sensor_interface().get_sensor_reading(
        ZED_IMU_PRIM_PATH,
        use_latest_data=True,
    )
    print(
        "[ZED_X_IMU]: "
        f"step={step} valid={bool(reading.is_valid)} time={float(reading.time):.6f} "
        f"lin_acc=({float(reading.lin_acc_x):.5f},"
        f"{float(reading.lin_acc_y):.5f},{float(reading.lin_acc_z):.5f}) "
        f"ang_vel=({float(reading.ang_vel_x):.5f},"
        f"{float(reading.ang_vel_y):.5f},{float(reading.ang_vel_z):.5f})",
        flush=True,
    )


def print_arm_diagnostics(robot, target_joint_pos: torch.Tensor, elapsed: float) -> None:
    leg_joint_names = [*LEG_HIP_JOINTS, *LEG_THIGH_JOINTS, *LEG_CALF_JOINTS]
    leg_joint_ids = [robot.joint_names.index(name) for name in leg_joint_names]
    arm_joint_ids = [robot.joint_names.index(name) for name in ARM_JOINTS]
    leg_pos = robot.data.joint_pos[:, leg_joint_ids]
    leg_vel = robot.data.joint_vel[:, leg_joint_ids]
    leg_target = target_joint_pos[:, leg_joint_ids]
    leg_error = leg_pos - leg_target
    leg_torque = robot.data.applied_torque[:, leg_joint_ids]
    arm_pos = robot.data.joint_pos[:, arm_joint_ids]
    arm_vel = robot.data.joint_vel[:, arm_joint_ids]
    arm_target = target_joint_pos[:, arm_joint_ids]
    arm_error = arm_pos - arm_target
    arm_torque = robot.data.applied_torque[:, arm_joint_ids]
    print(
        "[DIAG]: "
        f"t={elapsed:.2f}s "
        f"leg_abs_err_max={leg_error.abs().max().item():.4f}rad "
        f"leg_abs_vel_max={leg_vel.abs().max().item():.4f}rad/s "
        f"leg_abs_tau_max={leg_torque.abs().max().item():.2f}Nm "
        f"arm_abs_err_max={arm_error.abs().max().item():.4f}rad "
        f"arm_abs_vel_max={arm_vel.abs().max().item():.4f}rad/s "
        f"arm_abs_tau_max={arm_torque.abs().max().item():.2f}Nm",
        flush=True,
    )


def print_policy_diagnostics(controller: B2ArxIsaacPolicyController, elapsed: float) -> None:
    if not args_cli.print_policy_debug:
        return
    raw = controller.last_raw_action
    target = controller.last_q_target
    cmd = controller.last_command
    root_pos = controller.robot.data.root_pos_w[0].detach().cpu().numpy()
    root_quat_wxyz = controller.robot.data.root_quat_w[0].detach().cpu().numpy()
    qw, qx, qy, qz = (float(value) for value in root_quat_wxyz)
    root_yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    print(
        "[POLICY]: "
        f"t={elapsed:.2f}s state={controller.state_name} "
        f"cmd=[{cmd.vx:.2f} {cmd.vy:.2f} {cmd.wz:.2f}] "
        f"base=[{root_pos[0]:.3f} {root_pos[1]:.3f} {root_pos[2]:.3f} {root_yaw:.3f}] "
        f"ee_hold=[-{int(cmd.ee_step_negative_held)} +{int(cmd.ee_step_positive_held)}] "
        f"ee_event=[cycle={int(cmd.ee_cycle_dim)} step={cmd.ee_step} reset={int(cmd.ee_reset)}] "
        f"raw_abs_max={np.max(np.abs(raw)):.4f} "
        f"target_arm={np.array2string(target[12:18], precision=3, suppress_small=True)} "
        f"ee_sphere={np.array2string(np.array(controller.command_buffer.get()), precision=3, suppress_small=True)}",
        flush=True,
    )


def _policy_target_tensor(robot, controller: B2ArxIsaacPolicyController) -> torch.Tensor:
    target = robot.data.default_joint_pos.clone()
    controlled_ids = controller.plant.joint_ids
    target[:, controlled_ids] = torch.as_tensor(controller.last_q_target, dtype=target.dtype, device=target.device).reshape(1, -1)
    return target


def make_policy_controller(robot) -> B2ArxIsaacPolicyController:
    cfg = DEPLOY_CFG if DEPLOY_CFG is not None else load_deploy_config(args_cli.deploy_config)
    onnx = cfg.policy.resolved_onnx()
    deploy_yaml = cfg.policy.resolved_deploy_yaml()
    if not Path(onnx).exists():
        raise FileNotFoundError(f"Policy ONNX not found: {onnx}")
    if not Path(deploy_yaml).exists():
        raise FileNotFoundError(f"Policy deploy.yaml not found: {deploy_yaml}")
    manifest_values = None
    if cfg.policy.manifest is not None:
        manifest_values = verify_policy_bundle(cfg.policy)
    source = make_command_source(cfg.input, cfg.deploy)
    controller = B2ArxIsaacPolicyController(
        robot,
        deploy_yaml=deploy_yaml,
        onnx_path=onnx,
        start_state=cfg.deploy.start_state,
        command_source=source,
        ee_sphere=cfg.deploy.ee_sphere,
        auto_arm_loco=cfg.deploy.auto_arm_loco,
        arm_ema_tau=cfg.deploy.arm_ema_tau,
    )
    controller.reset()
    print(
        "[INFO]: Policy controller loaded: "
        f"config={args_cli.deploy_config} backend={cfg.input.backend} "
        f"start_state={cfg.deploy.start_state} auto_arm_loco={cfg.deploy.auto_arm_loco} "
        f"control_dt={controller.control_dt:.4f}s arm_ema_tau={cfg.deploy.arm_ema_tau:.4f}s",
        flush=True,
    )
    if cfg.policy.checkpoint is not None:
        print(f"[INFO]: Policy source checkpoint metadata: {cfg.policy.checkpoint}", flush=True)
    if cfg.policy.manifest is not None:
        print(f"[INFO]: Policy export manifest metadata: {cfg.policy.manifest}", flush=True)
    if manifest_values is not None:
        print(
            "[INFO]: Policy bundle SHA256 verified: "
            f"checkpoint={manifest_values['checkpoint_sha256']} "
            f"onnx={manifest_values['policy_full_onnx_sha256']} "
            f"deploy={manifest_values['deploy_yaml_sha256']}",
            flush=True,
        )
    return controller


# The standalone loop below advances one 200 Hz physics step explicitly and
# flushes Hydra/RTX rendering every four steps.  The rendering interval remains
# four so the Kit/RTX clock matches that 50 Hz flush cadence.  Crucially, a
# render flush never replaces the physics step on that iteration.
RENDER_DECIMATION = 4  # 200 Hz physics, 50 Hz renderer flush


def _needs_camera_frame(count: int) -> bool:
    """True when a renderer-side ZED probe frame is about to be saved."""
    return args_cli.save_zed_frames and (count == 30 or (count > 0 and count % 300 == 0))


def run_simulator(sim: SimulationContext, scene: InteractiveScene) -> None:
    sim_dt = sim.get_physics_dt()
    count = 0
    elapsed = 0.0
    reset_scene(scene)
    robot = scene["robot"]
    hold_joint_pos = robot.data.default_joint_pos.clone()
    hold_joint_vel = robot.data.default_joint_vel.clone()
    zero_joint_effort = hold_joint_pos.new_zeros(hold_joint_pos.shape)
    policy_controller = make_policy_controller(robot) if args_cli.control_mode == "policy" else None
    set_viewport_camera()
    print("[INFO]: B2+ARX R5 manipulation scene is running.", flush=True)
    if SELECTED_ENVIRONMENT_USD:
        print(f"[INFO]: Background environment USD: {SELECTED_ENVIRONMENT_USD}", flush=True)
        print_environment_physics_debug()
    else:
        print("[INFO]: Background environment: minimal Isaac ground plane.", flush=True)
    print(f"[INFO]: Control mode: {args_cli.control_mode}", flush=True)
    print(f"[INFO]: Arm gain profile: {args_cli.arm_gain_profile}", flush=True)
    if args_cli.control_mode == "hold":
        print("[INFO]: Hold pose: B2 training stance, R5 arm [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]", flush=True)
    else:
        print("[INFO]: Policy mode uses the sim2sim2real deployment FSM: Passive -> FixStand -> ArmPreAlign -> ArmLoco.", flush=True)
    if args_cli.no_zed:
        print("[INFO]: Embedded ZED X data path is disabled; its core-USD model remains loaded.", flush=True)
    else:
        print(
            "[INFO]: Using the core-USD embedded ZED X; "
            "ZED SDK simulation stream is " + ("disabled" if args_cli.no_zed_sdk_stream else "active"),
            flush=True,
        )
        if args_cli.save_zed_frames:
            print("[INFO]: Renderer probes: scene['zed_left_camera'], scene['zed_right_camera']", flush=True)
        print_zed_debug()
        print_zed_camera_world_poses()
    if args_cli.wrist_camera:
        print(
            "[INFO]: Wrist RGB-D uses the embedded official RealSense color "
            "camera with one shared render product: rgb8 + aligned 32FC1 metres.",
            flush=True,
        )

    try:
        _dense_n = int(os.environ.get("DIAG_EVERY", "0"))  # >0: print leg/arm diag every N physics steps
        while simulation_app.is_running():
            if args_cli.duration > 0.0 and elapsed >= args_cli.duration:
                break
            if _dense_n and count % _dense_n == 0:
                target_for_diag = hold_joint_pos if policy_controller is None else _policy_target_tensor(robot, policy_controller)
                print_arm_diagnostics(robot, target_for_diag, elapsed)
            if count % 200 == 0:
                print(f"[INFO]: t={elapsed:.2f}s", flush=True)
                target_for_diag = hold_joint_pos if policy_controller is None else _policy_target_tensor(robot, policy_controller)
                print_arm_diagnostics(robot, target_for_diag, elapsed)
                if policy_controller is not None:
                    print_policy_diagnostics(policy_controller, elapsed)
                print_ee_target_debug(scene, elapsed)

            if policy_controller is None:
                robot.set_joint_position_target(hold_joint_pos)
                robot.set_joint_velocity_target(hold_joint_vel)
                robot.set_joint_effort_target(zero_joint_effort)
            else:
                policy_controller.update(sim_dt)

            scene.write_data_to_sim()
            # Render only at the control rate (every RENDER_DECIMATION physics
            # steps).  ``SimulationContext.step(render=True)`` is intentionally
            # not used here: with a non-unit rendering interval the Kit loop
            # owns the substep clock and can advance it independently of this
            # explicit 200 Hz controller loop.  A physics-only step followed by
            # a separate render flush keeps /clock, ZED and RTX LiDAR on the
            # same simulation time.  Headless still renders when a renderer-side
            # ZED probe needs a fresh frame.
            render_this_step = (count % RENDER_DECIMATION == 0) or _needs_camera_frame(count)
            sim.step(render=False)
            if render_this_step:
                sim.render()
                report_zed_stream_status(zed_stream_logger_state)
            scene.update(sim_dt)
            if count == 100:
                print_zed_imu_reading(count)
            save_due = args_cli.save_zed_frames and (count == 30 or (count > 0 and count % 300 == 0))
            if save_due:
                save_zed_frame(scene, count)

            count += 1
            elapsed += sim_dt
        print(f"[INFO]: Simulation loop finished at t={elapsed:.3f}s after {count} steps.", flush=True)
    finally:
        if policy_controller is not None:
            policy_controller.close()


def main() -> None:
    robot_usd = Path(args_cli.robot_usd).expanduser().resolve()
    if not robot_usd.exists():
        raise FileNotFoundError(f"Robot USD not found: {robot_usd}")

    sim_cfg = sim_utils.SimulationCfg(
        dt=1.0 / 200.0,
        # The loop calls ``sim.step(render=False)`` once per physics step and
        # ``sim.render()`` separately every RENDER_DECIMATION steps.  Match the
        # renderer clock to that 50 Hz flush cadence without asking
        # ``step(render=True)`` to own or duplicate the controller substeps.
        render_interval=RENDER_DECIMATION,
        device=args_cli.device,
        use_fabric=not args_cli.disable_fabric,
    )
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.25, -2.0, 1.65], [0.55, 0.0, 0.45])

    scene_cfg = B2ArxManipulationSceneCfg(num_envs=args_cli.num_envs, env_spacing=args_cli.env_spacing)
    scene = InteractiveScene(scene_cfg)
    wrist_camera_prim_path = validate_core_sensor_prims(
        require_zed=not args_cli.no_zed,
        require_xt32=args_cli.ros2 and not args_cli.no_xt32_pointcloud,
        require_wrist_camera=args_cli.wrist_camera,
    )
    if not args_cli.no_zed and not args_cli.no_zed_sdk_stream:
        setup_zed_stream_graph(
            camera_prim_path=ZED_ASSET_PRIM_PATH,
            resolution=args_cli.zed_stream_resolution,
            fps=args_cli.zed_stream_fps,
            port=args_cli.zed_stream_port,
            transport=args_cli.zed_stream_transport,
        )
        print(
            "[INFO]: Official Stereolabs ZED SDK stream graph configured "
            "(streamer is not ready yet): "
            f"model=ZED_X resolution={args_cli.zed_stream_resolution} "
            f"fps={args_cli.zed_stream_fps} port={args_cli.zed_stream_port} "
            f"transport={args_cli.zed_stream_transport}",
            flush=True,
        )
        print(
            "[INFO]: Before starting zed_wrapper, wait for this process to print "
            "'[READY]: Official Stereolabs ZED Streamer initialized successfully with ID ...'.",
            flush=True,
        )
        if not args_cli.ros2:
            print(
                "[WARN]: --ros2 is disabled, so /clock is not published. "
                "zed_wrapper use_sim_time:=true requires launching this scene with --ros2.",
                flush=True,
            )

    if args_cli.ros2:
        import ros2_bridge
        ros2_bridge.setup_ros2_clock(domain_id=args_cli.ros2_domain_id)
        published_topics = [ros2_bridge.CLOCK_TOPIC]
        subscribed_topics = []

        if not args_cli.no_xt32_pointcloud:
            ros2_bridge.setup_ros2_xt32_pointcloud(
                domain_id=args_cli.ros2_domain_id,
                lidar_prim_path=XT32_LIDAR_PRIM_PATH,
                topic_name=args_cli.xt32_pointcloud_topic,
                frame_id=args_cli.xt32_frame_id,
                full_scan=True,
                show_debug_view=args_cli.show_xt32_debug_view,
            )
            published_topics.append(args_cli.xt32_pointcloud_topic)
            print(
                "[INFO]: Embedded official Hesai XT32 PointCloud2 graph configured: "
                f"prim={XT32_LIDAR_PRIM_PATH} topic={args_cli.xt32_pointcloud_topic} "
                f"frame={args_cli.xt32_frame_id} full_scan=True nominal_rate=10Hz",
                flush=True,
            )

        if args_cli.wrist_camera:
            if wrist_camera_prim_path is None:
                raise RuntimeError("Wrist camera prim validation returned no path")
            ros2_bridge.setup_ros2_wrist_camera(
                domain_id=args_cli.ros2_domain_id,
                camera_prim_path=wrist_camera_prim_path,
                width=args_cli.wrist_camera_width,
                height=args_cli.wrist_camera_height,
                frame_skip_count=args_cli.wrist_camera_frame_skip_count,
                frame_id=args_cli.wrist_camera_frame_id,
                color_topic=args_cli.wrist_camera_color_topic,
                color_info_topic=args_cli.wrist_camera_color_info_topic,
                aligned_depth_topic=args_cli.wrist_camera_aligned_depth_topic,
                aligned_depth_info_topic=args_cli.wrist_camera_aligned_depth_info_topic,
            )
            published_topics.extend(
                [
                    args_cli.wrist_camera_color_topic,
                    args_cli.wrist_camera_color_info_topic,
                    args_cli.wrist_camera_aligned_depth_topic,
                    args_cli.wrist_camera_aligned_depth_info_topic,
                ]
            )
            print(
                "[INFO]: Embedded wrist RealSense RGB-D graph configured: "
                f"prim={wrist_camera_prim_path} "
                f"resolution={args_cli.wrist_camera_width}x{args_cli.wrist_camera_height} "
                "rgb=rgb8 depth=32FC1(m) aligned_on=color_render_product",
                flush=True,
            )

        if args_cli.nav2:
            if DEPLOY_CFG is None or DEPLOY_CFG.input.backend != "ros2_twist":
                raise RuntimeError("--nav2 deploy config must use input.backend: ros2_twist")
            cmd_vel_topic = str(DEPLOY_CFG.input.ros2_twist["topic"])
            heartbeat_topic = str(DEPLOY_CFG.input.ros2_twist["heartbeat_topic"])
            ros2_bridge.setup_ros2_cmd_vel_subscriber(
                domain_id=args_cli.ros2_domain_id,
                topic_name=cmd_vel_topic,
                heartbeat_topic_name=heartbeat_topic,
            )
            subscribed_topics.extend([cmd_vel_topic, heartbeat_topic])

        print(
            f"[INFO]: ROS2 bridge active, domain={args_cli.ros2_domain_id}, "
            f"publishing {', '.join(published_topics)}"
            + (f", subscribing {', '.join(subscribed_topics)}" if subscribed_topics else ""),
            flush=True,
        )

    sim.reset()
    report_zed_stream_status(zed_stream_logger_state)
    try:
        run_simulator(sim, scene)
    finally:
        # Stereolabs releases its render products, annotators and stream encoder
        # from the timeline STOP callback.  Stop explicitly before closing Kit
        # so short/headless runs do not leave the SDK stream alive at shutdown.
        sim.stop()
        simulation_app.update()


if __name__ == "__main__":
    try:
        main()
    finally:
        remove_zed_stream_status_logger(zed_stream_logger_state)
        simulation_app.close()
