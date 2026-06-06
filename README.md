# B2ARX Isaac Sim

Isaac Lab scene workspace for B2 + ARX R5 embodied manipulation simulation.

## Local Paths

- Isaac Lab: `/home/lbz/IsaacLab`
- Isaac training workspace: `/home/lbz/b2arx`
- Local merged robot asset: `assets/my_B2Arx`
- Default robot USD: `assets/my_B2Arx/my_b2arx/my_robot.usd`
- Merged URDF source: `assets/my_B2Arx/my_robot.urdf`
- Conda environment: `isaaclab`

## Run Scene

```bash
cd /home/lbz/b2arx_isaac_sim
conda activate isaaclab
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --enable_cameras
```

The robot is held at its initial pose with PD drives so the B2+R5 does not collapse under gravity.
The default hold pose matches the Isaac training/sim2sim contract, while the arm PD gains default to the latest identified pure-PD fit:

- B2 stance: `[+0.15, +0.67, -1.32]` on FL/RL and `[-0.15, +0.67, -1.32]` on FR/RR
- R5 arm: `[0.0, 1.0, 0.8, 0.0, 0.0, 0.0]`
- Physics step: `dt=0.005` / 200 Hz
- Leg PD: hip/thigh `kp=300,kd=7.5`, calf `kp=500,kd=12.5`
- Arm PD default: `--arm_gain_profile identified`, using `/home/lbz/arx_actuator_identification/arx_id_data/20260605_001412/fit_out/actuator_params_isaac.yaml`

To compare against the original training high-gain arm profile:

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --enable_cameras --arm_gain_profile train
```

If the arm shakes, first test the default `identified` profile without cameras:

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --headless --duration 1.0 --no_scene_camera
```

To isolate contact issues from the table or objects, run only the robot and ground:

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --headless --duration 1.0 --no_scene_camera --no_workspace
```

The scene continuously commands the default joint positions, zero joint velocity targets, and zero feed-forward effort targets every physics step.
The terminal also prints arm diagnostics every simulated second:

- `arm_abs_err_max`: max absolute arm joint position error from the hold target
- `arm_abs_vel_max`: max absolute arm joint velocity
- `arm_abs_tau_max`: max absolute applied arm torque

The workspace table is placed in front of the robot instead of overlapping the B2 front legs. If `--no_workspace` is stable but the full scene shakes, the remaining issue is a contact/collision placement problem rather than the arm PD loop.

The simulated D435i RGB-D camera is mounted on the R5 wrist at `R5a_link6`. The viewport stays on the external scene camera by default so the wrist-mounted camera body is visible. To look through the wrist camera instead:

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --enable_cameras --viewer_camera d435i
```

To also save camera frames:

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --enable_cameras --save_camera_frames
```

Saved frames are written to `outputs/camera`.

## D435i Model

The D435i body is now part of the merged robot asset:

- URDF source: `assets/my_B2Arx/my_robot.urdf`
- D435 mesh: `assets/my_B2Arx/meshes/d435.dae`
- Generated USD: `assets/my_B2Arx/my_b2arx/my_robot.usd`

The URDF uses one fixed joint for the wrist camera assembly:

- `d435i_mount`: `R5a_link6` to `d435i_link`

This joint origin is the main manual alignment knob. During USD conversion with fixed-joint merging, the D435i body is merged into `R5a_link6`, so the scene does not spawn a separate `d435i_visual` asset.

Isaac still separates the rendered/simulated RGB-D sensor from the physical-looking camera body. The D435i shell comes from the robot USD, while `d435i_camera` is the actual `CameraCfg`/`PinholeCameraCfg` RGB-D sensor.

Regenerate the merged robot USD after editing the URDF or meshes:

```bash
cd /home/lbz/b2arx_isaac_sim
conda activate isaaclab
mkdir -p assets/my_B2Arx/my_b2arx
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p /home/lbz/IsaacLab/scripts/tools/convert_urdf.py \
  assets/my_B2Arx/my_robot.urdf \
  assets/my_B2Arx/my_b2arx/my_robot.usd \
  --merge-joints \
  --joint-target-type none \
  --headless
```

Headless smoke test:

```bash
cd /home/lbz/b2arx_isaac_sim
conda activate isaaclab
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --headless --duration 1.0 --no_scene_camera
```

If Isaac startup is very slow, check whether another Isaac training process is already consuming the GPU/CPU:

```bash
ps -ef | grep -E 'isaaclab|isaac_b2arx|fit_all_isaac' | grep -v grep
```

The first scene contains:

- B2 + ARX R5 loaded from the converted USD asset
- Ground plane
- Static work table
- Three colored rigid objects for perception/grasping tests
- Merged D435i body mounted on `R5a_link6`
- Optional wrist RGB-D camera sensor
