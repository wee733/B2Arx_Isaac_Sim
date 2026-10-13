# Policy Profiles

![Robotics: B2ARX](https://img.shields.io/badge/Robotics-B2ARX-475569)
![Configuration: YAML](https://img.shields.io/badge/Configuration-YAML-CB171E)
![Policy: Locomotion](https://img.shields.io/badge/Policy-Locomotion-7B42BC)

![B2ARX project concept illustration](../../assets/readme/b2arx-project-cover.png)

*AI-generated project concept illustration.*

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
