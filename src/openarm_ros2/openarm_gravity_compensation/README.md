# openarm_gravity_compensation

Gravity feedforward node for OpenArm bimanual ROS2 control.

## What this package does

- subscribes to /joint_states
- reads robot_description from robot_state_publisher
- computes gravity torque using KDL dynamics for left and right arm chains
- publishes feedforward effort commands to:
  - /left_gravity_compensation_controller/commands
  - /right_gravity_compensation_controller/commands

This package is designed to work with:

- trajectory controllers (main motion command)
- stiffness/damping controllers (main impedance)
- gravity feedforward effort controllers (assist term)

## Launch standalone

```bash
ros2 launch openarm_gravity_compensation gravity_compensation.launch.py
```

or with explicit params:

```bash
ros2 run openarm_gravity_compensation gravity_compensation_node \
  --ros-args --params-file /path/to/gravity_compensation.yaml
```

## Parameters

See config/gravity_compensation.yaml.

Important fields:

- publish_rate_hz
- robot_description_source_node
- left_arm.alpha / right_arm.alpha
- left_arm.root_link / left_arm.leaf_link
- right_arm.root_link / right_arm.leaf_link
- left_arm.command_topic / right_arm.command_topic

## Runtime checks

```bash
ros2 node list | grep gravity_compensation_node
ros2 topic hz /left_gravity_compensation_controller/commands
ros2 topic hz /right_gravity_compensation_controller/commands
```

## Typical integration path

Use MoveIt demo launch with gravity feedforward enabled:

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_gravity_compensation:=true
```
