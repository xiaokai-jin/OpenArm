# OpenArm Teleop

OpenArm supports 1:1 teleoperation from a leader arm to a follower arm in two control modes. See the [documentation](https://docs.openarm.dev/teleop/) for details.

## MoveIt Servo Continuous Teleop

A keyboard-based continuous Cartesian control script relies on ROS 2 **MoveIt Servo**. Instead of planning point-to-point motion, this script streams end-effector velocities in real time.

### Quick Start
1. Ensure the bimanual system is brought up with Servo capability:
   ```bash
   ros2 launch openarm_bimanual_moveit_config demo.launch.py use_servo:=true
   ```
2. Run the python interpreter node:
   ```bash
   python3 script/servo_keyboard.py
   ```

### Script Details (`script/servo_keyboard.py`)
- **Node Initialization:** Auto-calls the `start_servo` ROS service for both the left and right arms to transition them out of the default paused state.
- **Arm Toggling:** Hit `TAB` to switch control targets dynamically without restarting.
- **Fail-Safe Stopping:** Includes a `0.1s` select timeout. If no key triggers are observed (key released), the node immediately dispatches zero-velocity Twists to halt the robot, avoiding drift or run-away instances.

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/teleop/)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

Licensed under the Apache License 2.0. See [LICENSE.txt](LICENSE.txt) for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
