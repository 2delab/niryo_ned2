# Robot Movement Commands - Quick Index

This document provides quick access to all available ways to move the Niryo NED2/NED3pro robot to specific joint positions using ROS2.

## Quick Links

**Full Comprehensive Guide:** See `/home/i/niryo_ned2/MOVEMENT_COMMANDS_GUIDE.md` (386 lines, 13KB)

---

## Main Action Endpoints

### 1. RobotMove Action (RECOMMENDED - High-Level)
**Location:** `/niryo_robot_arm_commander/robot_action`

| Movement Type | cmd_type | Parameters | CLI Command |
|---|---|---|---|
| **Joint Positions** | 0 | `joints: [j1, j2, j3, j4, j5, j6]` | `ros2 action send_goal /niryo_robot_arm_commander/robot_action niryo_ned_ros2_interfaces/action/RobotMove "{cmd: {cmd_type: 0, joints: [0,0,0,0,0,0]}}"` |
| **TCP Pose (XYZ+RPY)** | 1 | `position: {x,y,z}` + `rpy: {roll,pitch,yaw}` | `ros2 action send_goal /niryo_robot_arm_commander/robot_action niryo_ned_ros2_interfaces/action/RobotMove "{cmd: {cmd_type: 1, position: {x: 0.3, y: 0, z: 0.3}, rpy: {roll: 0, pitch: 1.57, yaw: 0}}}"` |
| **Position Only (XYZ)** | 2 | `position: {x,y,z}` | `ros2 action send_goal /niryo_robot_arm_commander/robot_action niryo_ned_ros2_interfaces/action/RobotMove "{cmd: {cmd_type: 2, position: {x: 0.3, y: 0, z: 0.3}}}"` |
| **Orientation Only (RPY)** | 3 | `rpy: {roll,pitch,yaw}` | `ros2 action send_goal /niryo_robot_arm_commander/robot_action niryo_ned_ros2_interfaces/action/RobotMove "{cmd: {cmd_type: 3, rpy: {roll: 0, pitch: 1.57, yaw: 0}}}"` |
| **Pose with Quaternion** | 4 | `position: {x,y,z}` + `orientation: {x,y,z,w}` | `ros2 action send_goal /niryo_robot_arm_commander/robot_action niryo_ned_ros2_interfaces/action/RobotMove "{cmd: {cmd_type: 4, position: {x: 0.3, y: 0, z: 0.3}, orientation: {x: 0, y: 0.707, z: 0, w: 0.707}}}"` |
| **Linear Pose Move** | 5 | `position: {x,y,z}` + `rpy: {roll,pitch,yaw}` | Same as POSE but with linear interpolation |
| **Relative Shift** | 6 | `shift: {axis_number: 0-5, value: float}` | `ros2 action send_goal /niryo_robot_arm_commander/robot_action niryo_ned_ros2_interfaces/action/RobotMove "{cmd: {cmd_type: 6, shift: {axis_number: 0, value: 0.1}}}"` |
| **Waypoint Trajectory** | 8 | `list_poses: [...]` + `dist_smoothing: float` | See MOVEMENT_COMMANDS_GUIDE.md |
| **Draw Circle** | 10 | Special parameters | See MOVEMENT_COMMANDS_GUIDE.md |

### 2. FollowJointTrajectory Action (Low-Level)
**Location:** `/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory`
**Type:** `control_msgs/action/FollowJointTrajectory`

For executing precise multi-point trajectories with specific timing.

```bash
ros2 action send_goal /niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
      {positions: [j1, j2, j3, j4, j5, j6], time_from_start: {sec: 0, nanosec: 0}},
      {positions: [j1, j2, j3, j4, j5, j6], time_from_start: {sec: 2, nanosec: 0}}
    ]
  }}"
```

---

## Service Calls

| Service | Type | Purpose | CLI Command |
|---|---|---|---|
| **Forward Kinematics** | GetFK | Convert joints to TCP pose | `ros2 service call /niryo_robot/kinematics/forward niryo_ned_ros2_interfaces/srv/GetFK "{joints: [0, 0.57, 1.57, 0, 0, 0]}"` |
| **Inverse Kinematics** | GetIK | Convert TCP pose to joints | `ros2 service call /niryo_robot/kinematics/inverse niryo_ned_ros2_interfaces/srv/GetIK "{position: {x: 0.3, y: 0, z: 0.3}, rpy: {roll: 0, pitch: 1.57, yaw: 0}}"` |
| **Calibrate Motors** | SetInt | Calibrate robot motors | `ros2 service call /niryo_robot/joints_interface/calibrate_motors niryo_ned_ros2_interfaces/srv/SetInt "{value: 1}"` |

---

## Python Examples

### Simple Joint Move
```python
from rclpy.node import Node
from rclpy.action import ActionClient
from niryo_ned_ros2_interfaces.action import RobotMove

client = ActionClient(self, RobotMove, '/niryo_robot_arm_commander/robot_action')
goal = RobotMove.Goal()
goal.cmd.cmd_type = 0  # JOINTS
goal.cmd.joints = [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]
client.send_goal_async(goal)
```

### TCP Pose Move
```python
from geometry_msgs.msg import Point
from niryo_ned_ros2_interfaces.msg import RPY

goal = RobotMove.Goal()
goal.cmd.cmd_type = 1  # POSE
goal.cmd.position = Point(x=0.3, y=0.0, z=0.3)
goal.cmd.rpy = RPY(roll=0.0, pitch=1.57, yaw=0.0)
client.send_goal_async(goal)
```

See MOVEMENT_COMMANDS_GUIDE.md for full examples.

---

## Message References

**Source Files:**
- `/home/i/niryo_ned2/src/ned-ros2-driver/niryo_ned_ros2_interfaces/action/RobotMove.action`
- `/home/i/niryo_ned2/src/ned-ros2-driver/niryo_ned_ros2_interfaces/msg/ArmMoveCommand.msg`
- `/home/i/niryo_ned2/src/ned-ros2-driver/niryo_ned_ros2_interfaces/msg/RPY.msg`
- `/home/i/niryo_ned2/src/ned-ros2-driver/niryo_ned_ros2_interfaces/msg/ShiftPose.msg`

**Key Message Fields:**

RPY (Roll-Pitch-Yaw):
```
roll: float64
pitch: float64
yaw: float64
```

ShiftPose (Relative movement):
```
axis_number: int32 (0-5 for each joint)
value: float64 (in meters for position, radians for rotation)
```

---

## Supported Namespaces

All commands support namespaced robots:
```bash
# Robot with namespace "robot1"
ros2 action send_goal /robot1/niryo_robot_arm_commander/robot_action ...
ros2 service call /robot1/niryo_robot/kinematics/forward ...
```

---

## Important Notes

1. **All joint angles are in radians**
2. **All distances are in meters**
3. **TCP position (x, y, z) is in meters**
4. **Rotations (roll, pitch, yaw) are in radians**
5. **Actions return status and message on completion**
6. **Multi-point trajectories require timing for each point**

---

## Related Documentation

- **Full Guide:** `/home/i/niryo_ned2/MOVEMENT_COMMANDS_GUIDE.md`
- **Driver README:** `/home/i/niryo_ned2/src/ned-ros2-driver/README.md`
- **Official Docs:** https://niryorobotics.github.io/ned_ros/

---

## Sources Searched

- ned-ros2-driver repository
- ROS2 interface definitions (.action, .srv, .msg files)
- Integration and unit tests
- Official Niryo ROS documentation
- Action and service implementationss


