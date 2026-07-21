# Policy Profiles

The YAML files in this directory are runtime-selectable policy profiles. They
keep model paths, action safety limits, FSM settings, and ROS Twist settings out
of Python code.

`basic_locomotion.yaml` is the default for `--nav2`. To switch models, copy it,
point the `policy` entries at a complete directory under `models/`, set the
command bounds to the new training contract, and launch with
`--nav2 --deploy_config config/policies/<profile>.yaml`.

Navigation parameters remain in
`ros_ws/src/b2arx_nav2_bringup/config/b2arx_nav2.yaml`; ZED/Nvblox parameters
are in the adjacent YAML files in that package.
