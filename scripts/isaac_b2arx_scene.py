from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

from isaaclab.app import AppLauncher


DEFAULT_ROBOT_USD = Path(__file__).resolve().parents[1] / "assets" / "my_B2Arx" / "my_b2arx" / "my_robot.usd"
OUTPUT_DIR = Path(__file__).resolve().parents[1] / "outputs" / "camera"

# tag36h11:0 纹理 (nearest 放大到 800x800, 黑白边界锐利, 贴大面不糊; 见 assets/apriltag/)。
APRILTAG_TEXTURE_PATH = Path(__file__).resolve().parents[1] / "assets" / "apriltag" / "tag36_11_00000_800.png"


parser = argparse.ArgumentParser(description="Spawn a B2+ARX R5 manipulation scene in Isaac Lab.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of scene copies.")
parser.add_argument("--env_spacing", type=float, default=4.0, help="Spacing between cloned environments.")
parser.add_argument("--robot_usd", type=str, default=str(DEFAULT_ROBOT_USD), help="Path to the B2+R5 robot USD.")
parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means run until the app closes.")
parser.add_argument("--no_workspace", action="store_true", help="Do not spawn the table and manipulation objects.")
parser.add_argument("--no_scene_camera", action="store_true", help="Do not spawn the RGB-D scene camera.")
parser.add_argument("--save_camera_frames", action="store_true", help="Save D455 RGB/depth frames under outputs/camera.")
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
    choices=("scene", "d455", "rgb", "color", "depth", "infra1", "infra2", "left", "right"),
    default="scene",
    help="'d455'/'rgb' are aliases for color; 'left'/'right' are aliases for infra1/infra2.",
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
parser.add_argument("--print_d455_debug", action="store_true", help="Print official D455 asset/camera paths.")
parser.add_argument(
    "--show_depth_preview",
    action="store_true",
    help="Show a live OpenCV pseudo-color preview of scene['d455_depth_camera'].",
)
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric API and use USD instead.")
parser.add_argument("--ros2", action="store_true",
                    help="启用 ROS2 OmniGraph 发布(D455 color+camera_info+clock)+订阅(/tf 驱动 TagMarker)。")
parser.add_argument("--ros2_domain_id", type=int, default=23,
                    help="ROS_DOMAIN_ID, 必须与 Thor 端一致 (spec §4)。")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if not args_cli.no_scene_camera:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Enable the ROS2 bridge extension right after the app launches, before the physics
# device/backend is negotiated (SimulationContext). Enabling it later — between
# SimulationContext and InteractiveScene — leaves scene.env_origins on CPU while the
# robot lands on cuda:0, crashing reset_scene with a device mismatch. Official IsaacLab
# scripts (e.g. benchmarks/benchmark_non_rl.py) enable extensions at this same point.
if args_cli.ros2:
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")

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
from omni.physx.scripts import utils as physx_utils
from pxr import PhysxSchema, Usd, UsdPhysics

from d455_geometry import (
    D455_IMAGE_HEIGHT,
    D455_IMAGE_WIDTH,
    D455_MOUNT_POS,
    D455_MOUNT_ROT,
    D455_OFFICIAL_CAMERA_PRIMS,
    resolve_d455_usd_path,
)
from ee_sphere import target_world_to_sphere
from policy_deploy.command_sources import make_command_source
from policy_deploy.deploy_config import load_deploy_config
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
    "R5a_joint2": 1.0,
    "R5a_joint3": 0.8,
    "R5a_joint4": 0.0,
    "R5a_joint5": 0.0,
    "R5a_joint6": 0.0,
    "R5a_joint7": 0.0,
    "R5a_joint8": 0.0,
}

# The official rsd455.usd entity is spawned under R5a_link6/D455; its internal camera
# prims (under RSD455) carry the correct optical orientation, so binding CameraCfg to
# them (spawn=None) gives the right view without recomputing rotations.
D455_ASSET_PRIM_PATH = "/World/envs/env_0/Robot/R5a_link6/D455"
D455_CAMERA_ROOT_PRIM_PATH = f"{D455_ASSET_PRIM_PATH}/RSD455"
D455_COLOR_CAMERA_PRIM_PATH = f"{D455_CAMERA_ROOT_PRIM_PATH}/{D455_OFFICIAL_CAMERA_PRIMS['color']}"
D455_DEPTH_CAMERA_PRIM_PATH = f"{D455_CAMERA_ROOT_PRIM_PATH}/{D455_OFFICIAL_CAMERA_PRIMS['depth']}"
D455_INFRA1_CAMERA_PRIM_PATH = f"{D455_CAMERA_ROOT_PRIM_PATH}/{D455_OFFICIAL_CAMERA_PRIMS['infra1']}"
D455_INFRA2_CAMERA_PRIM_PATH = f"{D455_CAMERA_ROOT_PRIM_PATH}/{D455_OFFICIAL_CAMERA_PRIMS['infra2']}"
D455_CAMERA_PRIM_PATHS = {
    "d455": D455_COLOR_CAMERA_PRIM_PATH,
    "rgb": D455_COLOR_CAMERA_PRIM_PATH,
    "color": D455_COLOR_CAMERA_PRIM_PATH,
    "depth": D455_DEPTH_CAMERA_PRIM_PATH,
    "infra1": D455_INFRA1_CAMERA_PRIM_PATH,
    "infra2": D455_INFRA2_CAMERA_PRIM_PATH,
    "left": D455_INFRA1_CAMERA_PRIM_PATH,
    "right": D455_INFRA2_CAMERA_PRIM_PATH,
}

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


LOCAL_WAREHOUSE_USD = (
    "/home/lbz/Documents/isaac-sim-assets-environments-5.1.0/"
    "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
)

OFFICIAL_SCENE_ASSETS = {
    "grid": f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd",
    "rough_plane": f"{ISAAC_NUCLEUS_DIR}/Environments/Terrains/rough_plane.usd",
    "warehouse": f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd",
    "warehouse_local": LOCAL_WAREHOUSE_USD,
    "hospital": f"{ISAAC_NUCLEUS_DIR}/Environments/Hospital/hospital.usd",
}

# Repo-local rsd455.usd if present, else the Nucleus copy (see d455_geometry).
D455_USD_PATH = resolve_d455_usd_path(ISAAC_NUCLEUS_DIR)


def _resolve_usd_path(usd_path: str) -> str:
    """Resolve local USD paths while leaving Isaac/Nucleus URLs untouched."""
    if usd_path.startswith(("http://", "https://", "omniverse://")):
        return usd_path
    return str(Path(usd_path).expanduser().resolve())


# Load the deploy config once at module scope (policy mode only) so the scene cfg,
# built below, can honor scene.environment_usd. make_policy_controller reuses this.
DEPLOY_CFG = load_deploy_config(args_cli.deploy_config) if args_cli.control_mode == "policy" else None


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
        # Source:
        # /home/lbz/arx_actuator_identification/arx_id_data/20260605_001412/fit_out/actuator_params_isaac.yaml
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
                enabled_self_collisions=True,
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

    # Spawn the official rsd455.usd entity (repo-local copy) under R5a_link6, then bind a
    # CameraCfg(spawn=None) to each internal camera prim. The asset's internal prims carry
    # the correct optical orientation, so the wrist view is right without recomputing any
    # rotation. strip_d455_physics_apis() removes the asset's RigidBodyAPI/colliders after
    # spawn so it is a pure visual+sensor payload and not parsed as an articulation link.
    d455_asset = None if args_cli.no_scene_camera else AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Robot/R5a_link6/D455",
        spawn=sim_utils.UsdFileCfg(usd_path=D455_USD_PATH),
        init_state=AssetBaseCfg.InitialStateCfg(pos=tuple(D455_MOUNT_POS), rot=D455_MOUNT_ROT),
    )

    d455_color_camera = None if args_cli.no_scene_camera else CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/R5a_link6/D455/RSD455/Camera_OmniVision_OV9782_Color",
        update_period=1.0 / 30.0,
        height=D455_IMAGE_HEIGHT,
        width=D455_IMAGE_WIDTH,
        data_types=["rgb"],
        spawn=None,
    )

    d455_depth_camera = None if args_cli.no_scene_camera else CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/R5a_link6/D455/RSD455/Camera_Pseudo_Depth",
        update_period=1.0 / 30.0,
        height=D455_IMAGE_HEIGHT,
        width=D455_IMAGE_WIDTH,
        data_types=["distance_to_image_plane"],
        spawn=None,
    )

    d455_infra1_camera = None if args_cli.no_scene_camera else CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/R5a_link6/D455/RSD455/Camera_OmniVision_OV9782_Left",
        update_period=1.0 / 30.0,
        height=D455_IMAGE_HEIGHT,
        width=D455_IMAGE_WIDTH,
        data_types=["rgb"],
        spawn=None,
    )

    d455_infra2_camera = None if args_cli.no_scene_camera else CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/R5a_link6/D455/RSD455/Camera_OmniVision_OV9782_Right",
        update_period=1.0 / 30.0,
        height=D455_IMAGE_HEIGHT,
        width=D455_IMAGE_WIDTH,
        data_types=["rgb"],
        spawn=None,
    )

    # --- ROS2 回路 V1: 被检测的虚拟 AprilTag + 回流 marker (spec §2.3) ---
    # AprilTag 板用带显式 UV 的 quad mesh, 在 spawn_apriltag_board() 里建 (CuboidCfg 是
    # UsdGeom.Cube 没有 UV, OmniPBR 三平面投影在 0.1m 小面上贴不出 tag → 白板)。这里只留 marker。
    # ROS2SubscribeTransformTree 把 Thor 回流的 tag 位姿写到这个 prim (spec §1)。
    # 初始放在 (0,0,1.5) 离谱位置, 回路通时能明显看到它"跳"到 tag 处。
    tag_marker = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/TagMarker",
        spawn=sim_utils.SphereCfg(
            radius=0.03,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.8), roughness=0.4),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 1.5)),
    ) if args_cli.ros2 else None


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


def strip_d455_physics_apis() -> None:
    """Make the mounted D455 a pure visual/sensor payload, not a nested rigid body.

    The official rsd455.usd carries RigidBodyAPI/colliders. Spawned under the robot
    articulation, those would be parsed as articulation links and physx tensors would warn
    "did not match any rigid bodies" after they are removed. We recurse the WHOLE D455
    subtree and strip rigid/collision/joint APIs so it is a clean visual+sensor payload.
    """
    if args_cli.no_scene_camera:
        return
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    removed_rigid_count = 0
    removed_collider_count = 0
    disabled_joint_count = 0
    for env_index in range(args_cli.num_envs):
        root_path = f"/World/envs/env_{env_index}/Robot/R5a_link6/D455"
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim.IsValid():
            continue
        for prim in Usd.PrimRange(root_prim):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)
                removed_rigid_count += 1
            if prim.IsA(UsdPhysics.Joint):
                prim.GetAttribute("physics:jointEnabled").Set(False)
                disabled_joint_count += 1
            if prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                physx_utils.removeCollider(prim)
                removed_collider_count += 1
    if removed_rigid_count or removed_collider_count or disabled_joint_count:
        print(
            "[INFO]: D455 physics APIs stripped: "
            f"rigid={removed_rigid_count}, collider={removed_collider_count}, joints={disabled_joint_count}",
            flush=True,
        )


def spawn_apriltag_board() -> None:
    """Spawn a flat quad with explicit UVs per env and paint tag36h11:0 on it (--ros2 only).

    Why a hand-built mesh, not CuboidCfg: CuboidCfg makes a UsdGeom.Cube which has NO UV
    coords, so an OmniPBR diffuse_texture can only fall back to triplanar projection — on a
    0.1m face that shows a tiny crop and reads as blank. Here we build a UsdGeom.Mesh quad
    with st (UV) [0..1] mapped to the four corners, so the full tag maps exactly onto the
    face. size 0.1m must == Thor launch `size` (spec §4, R7). API (CreateMdlMaterialPrim /
    OmniPBR diffuse_texture / MaterialBindingAPI) verified vs isaacsim.replicator.behavior.
    """
    if not args_cli.ros2:
        return
    if not APRILTAG_TEXTURE_PATH.is_file():
        print(f"[WARN]: AprilTag texture not found: {APRILTAG_TEXTURE_PATH}", flush=True)
        return
    import omni.kit.commands
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    stage = omni.usd.get_context().get_stage()
    texture_asset = str(APRILTAG_TEXTURE_PATH)
    half = 0.05  # 0.1m tag, centered
    pos = Gf.Vec3d(1.16, 0.0, 0.455)
    spawned = 0
    for env_index in range(args_cli.num_envs):
        board_path = f"/World/envs/env_{env_index}/AprilTag"
        mesh = UsdGeom.Mesh.Define(stage, board_path)
        # Quad in the XY plane, facing +Z (up toward the wrist camera looking down).
        mesh.CreatePointsAttr([
            Gf.Vec3f(-half, -half, 0.0), Gf.Vec3f(half, -half, 0.0),
            Gf.Vec3f(half, half, 0.0), Gf.Vec3f(-half, half, 0.0),
        ])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateExtentAttr([Gf.Vec3f(-half, -half, 0.0), Gf.Vec3f(half, half, 0.0)])
        # Explicit UV (st) per face-vertex: full [0,1] square -> the whole tag on the face.
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
        )
        st.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
        UsdGeom.Xformable(mesh).AddTranslateOp().Set(pos)

        mtl_path = f"{board_path}/AprilTagMaterial"
        omni.kit.commands.execute(
            "CreateMdlMaterialPrim", mtl_url="OmniPBR.mdl", mtl_name="OmniPBR", mtl_path=mtl_path
        )
        mtl_prim = stage.GetPrimAtPath(mtl_path)
        shader = UsdShade.Shader(omni.usd.get_shader_from_material(mtl_prim, get_prim=True))
        # project_uvw=False -> use the mesh st coords we just authored (not triplanar).
        shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture_asset))
        shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(False)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(
            UsdShade.Material(mtl_prim), UsdShade.Tokens.strongerThanDescendants
        )
        spawned += 1
    if spawned:
        print(f"[INFO]: AprilTag tag36h11:0 board spawned on {spawned} env(s) at {tuple(pos)}.", flush=True)



def set_viewport_camera() -> None:
    if args_cli.headless:
        return
    if args_cli.viewer_camera == "scene":
        return
    if args_cli.no_scene_camera:
        print(f"[WARN]: --viewer_camera {args_cli.viewer_camera} was requested, but D455 cameras are disabled.")
        return
    try:
        from omni.kit.viewport.utility import get_active_viewport

        camera_path = D455_CAMERA_PRIM_PATHS[args_cli.viewer_camera]
        viewport = get_active_viewport()
        viewport.set_active_camera(camera_path)
        print(f"[INFO]: Viewport switched to {camera_path}", flush=True)
        if args_cli.viewer_camera == "depth":
            print(
                "[INFO]: Isaac viewport shows the depth camera's rendered view, not a depth colormap. "
                "Use --save_camera_frames and open depth_vis_*.png or depth_m_*.npy for depth values.",
                flush=True,
            )
    except Exception as exc:
        print(f"[WARN]: Could not switch viewport to {args_cli.viewer_camera} camera: {exc}", flush=True)


def _image_rgb_uint8(camera) -> np.ndarray | None:
    if "rgb" not in camera.data.output:
        return None
    rgb = camera.data.output["rgb"][0, ..., :3].detach().cpu().numpy()
    return np.clip(rgb, 0, 255).astype(np.uint8)


def _image_gray_uint8(camera) -> np.ndarray | None:
    rgb = _image_rgb_uint8(camera)
    if rgb is None:
        return None
    gray = 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
    return np.clip(gray, 0, 255).astype(np.uint8)


def _depth_meters(camera) -> np.ndarray | None:
    if "distance_to_image_plane" not in camera.data.output:
        return None
    depth = camera.data.output["distance_to_image_plane"][0].detach().cpu().numpy()
    if depth.ndim == 3:
        depth = depth[..., 0]
    return np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)


def _depth_visualization(depth: np.ndarray | None) -> np.ndarray | None:
    if depth is None:
        return None
    depth_vis = depth.copy()
    positive_depth = depth_vis[depth_vis > 0.0]
    if positive_depth.size:
        depth_vis = np.clip(depth_vis, 0.0, np.percentile(positive_depth, 98))
    return depth_vis


def _depth_preview_uint8(depth: np.ndarray | None) -> np.ndarray | None:
    if depth is None:
        return None
    positive_depth = depth[depth > 0.0]
    if positive_depth.size == 0:
        return None
    min_depth = float(np.percentile(positive_depth, 1))
    max_depth = float(np.percentile(positive_depth, 98))
    if max_depth <= min_depth:
        return None
    normalized = (np.clip(depth, min_depth, max_depth) - min_depth) / (max_depth - min_depth)
    return np.clip(255.0 * normalized, 0.0, 255.0).astype(np.uint8)


def update_depth_preview(scene: InteractiveScene) -> None:
    if args_cli.no_scene_camera or not args_cli.show_depth_preview:
        return
    if args_cli.headless:
        return
    try:
        import cv2
    except ImportError:
        if not getattr(update_depth_preview, "_warned_no_cv2", False):
            print("[WARN]: --show_depth_preview requested, but cv2 is not available.", flush=True)
            update_depth_preview._warned_no_cv2 = True
        return

    depth = _depth_meters(scene["d455_depth_camera"])
    preview = _depth_preview_uint8(depth)
    if preview is None:
        return
    colored = cv2.applyColorMap(preview, cv2.COLORMAP_TURBO)
    cv2.imshow("D455 depth_vis distance_to_image_plane", colored)
    cv2.waitKey(1)


def save_d455_frame(scene: InteractiveScene, count: int) -> None:
    if args_cli.no_scene_camera or not args_cli.save_camera_frames:
        return

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    saved = []

    color = _image_rgb_uint8(scene["d455_color_camera"])
    if color is not None:
        path = OUTPUT_DIR / f"color_{count:06d}.png"
        plt.imsave(path, color)
        saved.append(path.name)

    for name in ("infra1", "infra2"):
        gray = _image_gray_uint8(scene[f"d455_{name}_camera"])
        if gray is None:
            continue
        path = OUTPUT_DIR / f"{name}_{count:06d}.png"
        plt.imsave(path, gray, cmap="gray", vmin=0, vmax=255)
        saved.append(path.name)

    depth = _depth_meters(scene["d455_depth_camera"])
    if depth is not None:
        path = OUTPUT_DIR / f"depth_m_{count:06d}.npy"
        np.save(path, depth)
        saved.append(path.name)

    depth_vis = _depth_visualization(depth)
    if depth_vis is not None:
        path = OUTPUT_DIR / f"depth_vis_{count:06d}.png"
        plt.imsave(path, depth_vis, cmap="turbo")
        saved.append(path.name)

    if saved:
        print(f"[INFO]: Saved D455 frames to {OUTPUT_DIR}: {', '.join(saved)}", flush=True)


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


def print_d455_debug() -> None:
    if args_cli.no_scene_camera or not args_cli.print_d455_debug:
        return

    print("[D455]: Official Isaac Sim D455 USD asset is mounted under R5a_link6 (repo-local rsd455.usd).")
    print(
        "[D455]: "
        f"usd_path={D455_USD_PATH} "
        f"resolution={D455_IMAGE_WIDTH}x{D455_IMAGE_HEIGHT} "
        f"mount_pos={np.array2string(D455_MOUNT_POS, precision=4, suppress_small=True)} "
        f"mount_quat_wxyz={np.array2string(np.array(D455_MOUNT_ROT), precision=6, suppress_small=True)}",
        flush=True,
    )
    for stream, prim_name in D455_OFFICIAL_CAMERA_PRIMS.items():
        print(f"[D455]: {stream:6s} camera prim = {{ENV_NS}}/Robot/R5a_link6/D455/RSD455/{prim_name}", flush=True)


def print_d455_camera_world_poses() -> None:
    """Read-only diagnostic: measure each camera prim's TRUE pose in the live scene.

    This isolates whether depth/infra cameras really sit below color (a real vertical
    offset in R5a_link6 frame) or only appear lower because the left/right stereo
    baseline projects diagonally on screen from the default viewport angle.
    """
    if args_cli.no_scene_camera or not args_cli.print_d455_debug:
        return
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    link6_path = "/World/envs/env_0/Robot/R5a_link6"
    link6_prim = stage.GetPrimAtPath(link6_path)
    if not link6_prim.IsValid():
        print(f"[D455_POSE]: R5a_link6 not found at {link6_path}", flush=True)
        return
    link6_w = UsdGeom.Xformable(link6_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    link6_w_inv = link6_w.GetInverse()

    world = {}
    link6 = {}
    for stream, prim_name in D455_OFFICIAL_CAMERA_PRIMS.items():
        cam_path = f"/World/envs/env_0/Robot/R5a_link6/D455/RSD455/{prim_name}"
        prim = stage.GetPrimAtPath(cam_path)
        if not prim.IsValid():
            print(f"[D455_POSE]: {stream:6s} prim NOT FOUND at {cam_path}", flush=True)
            continue
        cam_w = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        wt = cam_w.ExtractTranslation()
        world[stream] = np.array([wt[0], wt[1], wt[2]], dtype=np.float64)
        cam_in_link6 = cam_w * link6_w_inv
        lt = cam_in_link6.ExtractTranslation()
        link6[stream] = np.array([lt[0], lt[1], lt[2]], dtype=np.float64)

    if "color" in world:
        cw, cl = world["color"], link6["color"]
        print("[D455_POSE]: delta vs COLOR  (dz_link6==vertical in wrist frame; dz_world==vertical on screen)", flush=True)
        for stream in world:
            dw = world[stream] - cw
            dl = link6[stream] - cl
            print(
                f"[D455_POSE]: {stream:6s} "
                f"dz_link6={dl[2]:+.4f}m dy_link6={dl[1]:+.4f}m dx_link6={dl[0]:+.4f}m | "
                f"dz_world={dw[2]:+.4f}m",
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
    print(
        "[POLICY]: "
        f"t={elapsed:.2f}s state={controller.state_name} "
        f"cmd=[{cmd.vx:.2f} {cmd.vy:.2f} {cmd.wz:.2f}] "
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
    source = make_command_source(cfg.input, cfg.deploy)
    controller = B2ArxIsaacPolicyController(
        robot,
        deploy_yaml=deploy_yaml,
        onnx_path=onnx,
        start_state=cfg.deploy.start_state,
        command_source=source,
        ee_sphere=cfg.deploy.ee_sphere,
        auto_arm_loco=cfg.deploy.auto_arm_loco,
    )
    controller.reset()
    print(
        "[INFO]: Policy controller loaded: "
        f"config={args_cli.deploy_config} backend={cfg.input.backend} "
        f"start_state={cfg.deploy.start_state} auto_arm_loco={cfg.deploy.auto_arm_loco} "
        f"control_dt={controller.control_dt:.4f}s",
        flush=True,
    )
    if cfg.policy.checkpoint is not None:
        print(f"[INFO]: Policy source checkpoint metadata: {cfg.policy.checkpoint}", flush=True)
    if cfg.policy.manifest is not None:
        print(f"[INFO]: Policy export manifest metadata: {cfg.policy.manifest}", flush=True)
    return controller


RENDER_DECIMATION = 4  # render every N physics steps (200Hz/4 = 50Hz, == training render_interval)


def _needs_camera_frame(count: int) -> bool:
    """True on physics steps where a fresh rendered frame is consumed (preview/save)."""
    if args_cli.show_depth_preview and not args_cli.headless:
        return True
    if count == 30:
        return True
    if args_cli.save_camera_frames and count > 0 and count % 300 == 0:
        return True
    return False


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
        print("[INFO]: Hold pose: B2 training stance, R5 arm [0.0, 1.0, 0.8, 0.0, 0.0, 0.0]", flush=True)
    else:
        print("[INFO]: Policy mode uses the sim2sim2real deployment FSM: Passive -> FixStand -> ArmPreAlign -> ArmLoco.", flush=True)
    if args_cli.no_scene_camera:
        print("[INFO]: D455 cameras are disabled.", flush=True)
    else:
        print(
            "[INFO]: D455 camera entities: "
            "scene['d455_color_camera'], scene['d455_depth_camera'], "
            "scene['d455_infra1_camera'], scene['d455_infra2_camera']",
            flush=True,
        )
        print(
            "[INFO]: Depth data type: scene['d455_depth_camera'].data.output['distance_to_image_plane']",
            flush=True,
        )
        print_d455_debug()
        print_d455_camera_world_poses()
        if args_cli.show_depth_preview:
            if args_cli.headless:
                print("[WARN]: --show_depth_preview is ignored in --headless mode.", flush=True)
            else:
                print("[INFO]: Live depth preview window uses distance_to_image_plane, not viewport RGB.", flush=True)

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
            # Render only at the control rate (every `decimation` physics steps), like the
            # training env (render_interval=decimation). Rendering every 200Hz physics step
            # — especially with the D455 cameras and a heavy background scene — drops the
            # real-time factor far below 1 (slow-motion) and makes smooth motion look jittery
            # in the viewport. Headless still renders when cameras/preview need the frame.
            render_this_step = (count % RENDER_DECIMATION == 0) or _needs_camera_frame(count)
            sim.step(render=render_this_step)
            scene.update(sim_dt)
            update_depth_preview(scene)
            if count == 30 or (args_cli.save_camera_frames and count > 0 and count % 300 == 0):
                save_d455_frame(scene, count)

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

    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 200.0, device=args_cli.device, use_fabric=not args_cli.disable_fabric)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.25, -2.0, 1.65], [0.55, 0.0, 0.45])

    scene_cfg = B2ArxManipulationSceneCfg(num_envs=args_cli.num_envs, env_spacing=args_cli.env_spacing)
    scene = InteractiveScene(scene_cfg)
    strip_d455_physics_apis()
    spawn_apriltag_board()

    if args_cli.ros2:
        import ros2_bridge
        ros2_bridge.setup_d455_ros2_publishers(
            color_camera_prim_path=D455_COLOR_CAMERA_PRIM_PATH,
            domain_id=args_cli.ros2_domain_id,
            width=D455_IMAGE_WIDTH, height=D455_IMAGE_HEIGHT,
        )
        # 建 ROS 光学系子 prim (绕 X 180°) 当 TF parent, 再起订阅图。顺序: prim 必须先存在。
        optical_prim = ros2_bridge.setup_color_optical_frame_prim(
            color_camera_prim_path=D455_COLOR_CAMERA_PRIM_PATH,
        )
        ros2_bridge.setup_tag_tf_subscriber(
            domain_id=args_cli.ros2_domain_id, color_optical_prim_path=optical_prim,
        )
        print(f"[INFO]: ROS2 bridge active, domain={args_cli.ros2_domain_id}, "
              f"publishing {ros2_bridge.COLOR_IMAGE_TOPIC}", flush=True)

    sim.reset()
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
