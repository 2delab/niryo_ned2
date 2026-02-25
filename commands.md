

calibrate 

```bash
ros2 service call /niryo_robot/joints_interface/calibrate_motors niryo_ned_ros2_interfaces/srv/SetInt "{value: 1}"
```

set tcp

```bash
ros2 service call /niryo_robot_tools_commander/set_tcp niryo_ned_ros2_interfaces/srv/SetTCP \
  '{position: {x: 0.0726, y: -0.0007261, z: -0.01155}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}, tcp_version: 2}'
```


publish tf
```bash
ros2 run tf2_ros static_transform_publisher 0.3 0.0 0.2 0 0 0 world target
```

echo tf
```bash
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>

```

move to pose
```bash
  ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 1, position: {x: 0.3, y: 0.0, z: 0.2}, rpy: {roll: 0.0, pitch: 0.0, yaw: 0.0}}}"

```

move to pose linear

```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 5, position: {x: 0.3, y: 0.0, z: 0.2}, rpy: {roll: 0.0, pitch: 0.0, yaw: 0.0}}}"
```

move joints

```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 0, joints: [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]}}"
```

open gripper

```bash
ros2 action send_goal /niryo_robot_tools_commander/action_server \
  niryo_ned_ros2_interfaces/action/Tool \
  "{cmd: {cmd_type: 1, tool_id: 11, max_torque_percentage: 80, hold_torque_percentage: 30}}"

```

close gripper

```bash
ros2 action send_goal /niryo_robot_tools_commander/action_server \
  niryo_ned_ros2_interfaces/action/Tool \
  "{cmd: {cmd_type: 2, tool_id: 11, max_torque_percentage: 80, hold_torque_percentage: 30}}"
```

move xyz only


get_robot state
```bash
ros2 topic echo /niryo_robot/robot_state --once

```

forward kinematics
```bash
ros2 service call /niryo_robot/kinematics/forward \
  niryo_ned_ros2_interfaces/srv/GetFK \
  "{joints: [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]}"
```

inverse_kinematics
```bash
ros2 service call /niryo_robot/kinematics/inverse \
  niryo_ned_ros2_interfaces/srv/GetIK \
  "{pose: {position: {x: 0.3, y: 0.0, z: 0.3}, rpy: {roll: 0.0, pitch: 1.57, yaw: 0.0}}}"
```


camera link
```bash
ros2 run tf2_ros static_transform_publisher 0.0385 0 0 0 0 1.7453 wrist_link camera

```