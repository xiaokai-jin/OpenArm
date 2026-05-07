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
- `use_servo`: enable/disable MoveIt Servo nodes for real-time Cartesian teleoperation
- `gravity_compensation_params_file`: parameter file for gravity node
- `use_fake_hardware`: software simulation mode
- `right_can_interface` / `left_can_interface`: real hardware CAN mapping

### Expected control chain

1. `ros2_control_node` starts with bimanual hardware
2. trajectory controllers are activated (left/right)
3. MoveIt Servo nodes are optionally activated (if `use_servo:=true`)
4. stiffness and damping forward controllers are activated
5. left/right gravity effort controllers are activated
6. `gravity_compensation_node` publishes torque feedforward to both effort controllers

## Real-time Cartesian Teleoperation (MoveIt Servo)

You can smoothly drive the robot's TCP in Cartesian space (XYZ translation, RPY rotation) by enabling MoveIt Servo. 

### 1. Launch with Servo Enabled
By setting `use_servo:=true`, the launch file evaluates an `IfCondition` to spawn `servo_node_left` and `servo_node_right`. These load parameters from `config/servo_left.yaml` and `config/servo_right.yaml` respectively.

```sh
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_servo:=true
```

*(Note: Adding `use_fake_hardware:=true` is great for testing offline).*

### 2. Run the Keyboard Controller
Open a new terminal to publish Cartesian velocity commands (`TwistStamped`):
```sh
cd ~/OpenArm
source install/setup.bash
python3 src/openarm_teleop/script/servo_keyboard.py
```

### How it works 
1. The python script attempts to call `/servo_node_left(right)/start_servo` safely, activating both MoveIt Servo nodes.
2. The user inputs translate into 100Hz `TwistStamped` velocity frames fed into `~/delta_twist_cmds`.
3. If no key is pressed for `0.1s`, a zero-velocity vector ensures the robot stops immediately.
4. Singularity checks in the YAML configs are adjusted (`lower_singularity_threshold: 100.0`, `hard_stop_singularity_threshold: 130.0`) to avoid abrupt crashes near extended joint setups.

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
