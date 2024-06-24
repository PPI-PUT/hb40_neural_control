# hb40_neural_controller
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to hb40_neural_controller
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch hb40_neural_controller hb40_neural_controller.launch.py
```
check and change launch.py to use package with sim or robot

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name          | Type       | Description                                                                                                                              |
| ------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `bridge_data` | BridgeData | Messages contain the data send from MainBoard with update frames - actuator positions, velocities, torques as well as raw data from AHRS |
| `robot_state` | RobotState | Contains output of MAB state estimation, kinematics and dynamics.                                                                        |
| `cmd_vel`     | Twist      | Joystick msg                                                                                                                             |
| `system_cmd`  | String     | System msg                                                                                                                               |

### Output

| Name                  | Type           | Description                                                                                                                         |
| --------------------- | -------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| `joint_command`       | JointCommand   | Contains commands for robots actuators: target position, velocity and torque as well as positional gain (kp) and velocity gain (kd) |
| `debug/joint_command` | JointCommand   | Sample debug.                                                                                                                       |
| `debug/action`        | VectorFloatMsg | Sample debug.                                                                                                                       |
| `debug/tensor`        | VectorFloatMsg | Sample debug.                                                                                                                       |
