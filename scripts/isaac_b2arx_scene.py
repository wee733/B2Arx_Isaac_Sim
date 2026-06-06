from __future__ import annotations

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher


DEFAULT_ROBOT_USD = Path(__file__).resolve().parents[1] / "assets" / "my_B2Arx" / "my_b2arx" / "my_robot.usd"
OUTPUT_DIR = Path(__file__).resolve().parents[1] / "outputs" / "camera"


parser = argparse.ArgumentParser(description="Spawn a B2+ARX R5 manipulation scene in Isaac Lab.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of scene copies.")
parser.add_argument("--env_spacing", type=float, default=4.0, help="Spacing between cloned environments.")
parser.add_argument("--robot_usd", type=str, default=str(DEFAULT_ROBOT_USD), help="Path to the B2+R5 robot USD.")
parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means run until the app closes.")
parser.add_argument("--no_workspace", action="store_true", help="Do not spawn the table and manipulation objects.")
parser.add_argument("--no_scene_camera", action="store_true", help="Do not spawn the RGB-D scene camera.")
parser.add_argument("--save_camera_frames", action="store_true", help="Save D435i RGB/depth frames under outputs/camera.")
parser.add_argument(
    "--arm_gain_profile",
    choices=("identified", "train"),
    default="identified",
    help="Arm PD gains. 'identified' uses the 2026-06-05 pure-PD fit; 'train' uses the nominal Isaac training contract.",
)
parser.add_argument(
    "--viewer_camera",
    choices=("scene", "d435i"),
    default="scene",
    help="Viewport camera to use after startup. Use 'd435i' to look through the wrist RGB-D camera.",
)
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric API and use USD instead.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if not args_cli.no_scene_camera and not args_cli.enable_cameras:
    parser.error("D435i camera is enabled by default. Add --enable_cameras, or use --no_scene_camera.")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

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

D435I_CAMERA_PRIM_PATH = "/World/envs/env_0/Robot/R5a_link6/d435i_camera"

# The merged robot URDF fixes d435i_link directly to R5a_link6 at xyz=(0.06, 0, 0.10), rpy=(0, 0.523599, 0).
# The camera sensor is separate from the visible/colliding D435i body in the robot USD.
D435I_SENSOR_POS = (0.06, 0.0, 0.10)
# R5a_link6 -> d435i_mount pitch, composed with the camera optical-frame rotation.
D435I_SENSOR_ROT = (0.353553, -0.612372, 0.612372, -0.353553)

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

    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

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

    d435i_camera = None if args_cli.no_scene_camera else CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/R5a_link6/d435i_camera",
        update_period=1.0 / 30.0,
        height=480,
        width=640,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=1.93,
            focus_distance=0.6,
            horizontal_aperture=3.896,
            clipping_range=(0.05, 10.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=D435I_SENSOR_POS,
            rot=D435I_SENSOR_ROT,
            convention="ros",
        ),
    )


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


def set_viewport_camera() -> None:
    if args_cli.headless:
        return
    if args_cli.viewer_camera == "scene":
        return
    if args_cli.no_scene_camera:
        print("[WARN]: --viewer_camera d435i was requested, but the D435i camera is disabled.")
        return
    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        viewport.set_active_camera(D435I_CAMERA_PRIM_PATH)
        print(f"[INFO]: Viewport switched to {D435I_CAMERA_PRIM_PATH}")
    except Exception as exc:
        print(f"[WARN]: Could not switch viewport to D435i camera: {exc}")


def save_d435i_frame(scene: InteractiveScene, count: int) -> None:
    if args_cli.no_scene_camera or not args_cli.save_camera_frames:
        return

    camera = scene["d435i_camera"]
    if "rgb" not in camera.data.output or "distance_to_image_plane" not in camera.data.output:
        return

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    rgb = camera.data.output["rgb"][0, ..., :3].detach().cpu().numpy()
    depth = camera.data.output["distance_to_image_plane"][0].detach().cpu().numpy()
    if depth.ndim == 3:
        depth = depth[..., 0]

    rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
    depth_vis = depth.copy()
    positive_depth = depth_vis[depth_vis > 0.0]
    if positive_depth.size:
        depth_vis = np.clip(depth_vis, 0.0, np.percentile(positive_depth, 98))

    plt.imsave(OUTPUT_DIR / f"rgb_{count:06d}.png", rgb)
    plt.imsave(OUTPUT_DIR / f"depth_{count:06d}.png", depth_vis, cmap="turbo")
    print(f"[INFO]: Saved D435i frames to {OUTPUT_DIR}")


def print_arm_diagnostics(robot, hold_joint_pos: torch.Tensor, elapsed: float) -> None:
    arm_joint_ids = [robot.joint_names.index(name) for name in ARM_JOINTS]
    arm_pos = robot.data.joint_pos[:, arm_joint_ids]
    arm_vel = robot.data.joint_vel[:, arm_joint_ids]
    arm_target = hold_joint_pos[:, arm_joint_ids]
    arm_error = arm_pos - arm_target
    arm_torque = robot.data.applied_torque[:, arm_joint_ids]
    print(
        "[DIAG]: "
        f"t={elapsed:.2f}s "
        f"arm_abs_err_max={arm_error.abs().max().item():.4f}rad "
        f"arm_abs_vel_max={arm_vel.abs().max().item():.4f}rad/s "
        f"arm_abs_tau_max={arm_torque.abs().max().item():.2f}Nm"
    )


def run_simulator(sim: SimulationContext, scene: InteractiveScene) -> None:
    sim_dt = sim.get_physics_dt()
    count = 0
    elapsed = 0.0
    reset_scene(scene)
    robot = scene["robot"]
    hold_joint_pos = robot.data.default_joint_pos.clone()
    hold_joint_vel = robot.data.default_joint_vel.clone()
    zero_joint_effort = hold_joint_pos.new_zeros(hold_joint_pos.shape)
    set_viewport_camera()
    print("[INFO]: B2+ARX R5 manipulation scene is running.")
    print(f"[INFO]: Arm gain profile: {args_cli.arm_gain_profile}")
    print("[INFO]: Hold pose: B2 training stance, R5 arm [0.0, 1.0, 0.8, 0.0, 0.0, 0.0]")
    print("[INFO]: D435i camera is disabled." if args_cli.no_scene_camera else "[INFO]: RGB-D camera entity: scene['d435i_camera']")

    while simulation_app.is_running():
        if args_cli.duration > 0.0 and elapsed >= args_cli.duration:
            break
        if count % 200 == 0:
            print(f"[INFO]: t={elapsed:.2f}s")
            print_arm_diagnostics(robot, hold_joint_pos, elapsed)

        robot.set_joint_position_target(hold_joint_pos)
        robot.set_joint_velocity_target(hold_joint_vel)
        robot.set_joint_effort_target(zero_joint_effort)

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
        if count == 30 or (args_cli.save_camera_frames and count > 0 and count % 300 == 0):
            save_d435i_frame(scene, count)

        count += 1
        elapsed += sim_dt


def main() -> None:
    robot_usd = Path(args_cli.robot_usd).expanduser().resolve()
    if not robot_usd.exists():
        raise FileNotFoundError(f"Robot USD not found: {robot_usd}")

    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 200.0, device=args_cli.device, use_fabric=not args_cli.disable_fabric)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.25, -2.0, 1.65], [0.55, 0.0, 0.45])

    scene_cfg = B2ArxManipulationSceneCfg(num_envs=args_cli.num_envs, env_spacing=args_cli.env_spacing)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
