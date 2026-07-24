# EE Target Pipeline

## Scope

The manipulation scene converts a selected object's world position into the deploy-side EE sphere command.
The conversion is independent of perception, ROS transport, and Isaac sensor setup, so the same contract can
be reused by local debug targets or an external target-pose producer.

## EE Sphere Contract

The policy/deploy interface uses these command-buffer ranges:

- `r`: `0.30` to `0.60`
- `pitch`: `-0.50` to `1.00`
- `yaw`: `-1.50` to `1.50`
- initial command: `(0.36, 0.56, 0.0)`

The conversion mirrors the training helper:

1. Compute `sphere_center = base_xy + yaw_rotate([0.23, 0, 0]) + z=0.76`.
2. Convert the target point from world frame into the base yaw frame.
3. Convert yaw-frame Cartesian `(x, y, z)` to `(r, pitch, yaw)`.
4. Clamp the result to the deploy-side limits.

The pure conversion lives in `scripts/ee_sphere.py` and has no Isaac runtime dependency. The scene-side debug
path can be exercised with:

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --print_ee_target_debug \
  --target_object red_box
```

## ROS2 / Thor Boundary

Keep target estimation and policy deployment separated:

- An upstream component publishes a target pose in an explicitly named frame.
- A frame-aware adapter converts that pose into the robot/world frame expected by `target_world_to_sphere()`.
- The deploy controller consumes only the final EE sphere command and robot proprioception.

Two useful interface levels are:

```text
/b2arx/target_pose
  geometry_msgs/msg/PoseStamped
  semantic: target pose before conversion

/b2arx/ee_sphere_cmd
  geometry_msgs/msg/Vector3Stamped (or an equivalent typed command)
  semantic: converted (radius, pitch, yaw)
```

Never reinterpret a pose by changing only `header.frame_id`; transform it through the corresponding TF before
running the sphere conversion. This keeps target estimation, coordinate conversion, and policy execution as
independent replaceable layers.
