# MoveIt2 on Bimanual Openarms

Ensure the ROS2 packages and dependencies are installed by following the instructions in `openarm_ros2/README.md`.

## Physical Hardware
1. Run `init_can.sh` from `openarm_bringup/utils`. 
   By default, can0 is the right arm and can1 is the left arm, but this can be adjusted in the ros2_control definition in `openarm_description/urdf/openarm.ros2_control.xacro`.

2. Optionally, start the head-mounted realsense camera. This enables the octomap occupancy grid for planning around obstacles.
   
```sh
ros2 launch openarm_bimanual_bringup depth_camera.launch.py
```

## Launch the demo

```sh
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

## Gravity compensation feedforward

This package can launch gravity feedforward together with MoveIt execution.

### Launch with gravity feedforward enabled

```sh
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_gravity_compensation:=true
```

### Important launch arguments

- `use_gravity_compensation`: enable/disable gravity feedforward node and effort controllers
- `gravity_compensation_params_file`: parameter file for gravity node
- `use_fake_hardware`: software simulation mode
- `right_can_interface` / `left_can_interface`: real hardware CAN mapping

### Expected control chain

1. `ros2_control_node` starts with bimanual hardware
2. trajectory controllers are activated (left/right)
3. stiffness and damping forward controllers are activated
4. left/right gravity effort controllers are activated
5. `gravity_compensation_node` publishes torque feedforward to both effort controllers

### Verify gravity feedforward is alive

```sh
ros2 node list | grep gravity_compensation_node
ros2 control list_controllers | grep gravity_compensation
ros2 topic hz /left_gravity_compensation_controller/commands
ros2 topic hz /right_gravity_compensation_controller/commands
```

### Troubleshooting

- If arm drops with low kp/kd: gravity node likely not publishing.
- If controllers are active but no command topic frequency: check gravity node process log first.
- If planning executes but starts with a visible dip: increase gravity alpha carefully and retune kp/kd for shoulder/elbow joints.
