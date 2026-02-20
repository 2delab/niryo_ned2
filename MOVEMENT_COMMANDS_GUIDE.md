# Comprehensive Guide to Moving Niryo Robot to Joint Positions

## Overview
The Niryo NED2/NED3pro robot can be controlled via ROS2 using the ned-ros2-driver. There are multiple ways to move the robot to specific joint positions, ranging from CLI commands to action calls and Python code.

---

## 1. ROS2 ACTION CALLS (Recommended for Direct Control)

### 1.1 RobotMove Action (High-Level Arm Commander)

**Action Name:** `/niryo_robot_arm_commander/robot_action`
**ROS2 Type:** `niryo_ned_ros2_interfaces/action/RobotMove`
**ROS1 Type:** `niryo_robot_arm_commander/RobotMoveAction`

#### Move to Joint Positions (JOINTS Mode)

**CLI Syntax:**
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action niryo_ned_ros2_interfaces/action/RobotMove "{cmd: {cmd_type: 0, joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}"
```

**Parameters:**
- `cmd_type`: 0 (JOINTS constant)
- `joints`: Array of 6 float64 values (one for each joint in radians)
- `tcp_version`: 1 (LEGACY) or 2 (DH_CONVENTION) - optional

**Example - Move to Home Position:**
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 0, joints: [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]}}"
```

#### Move to TCP Pose (POSE Mode)

**CLI Syntax:**
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 1, position: {x: 0.3, y: 0.0, z: 0.3}, rpy: {roll: 0.0, pitch: 1.57, yaw: 0.0}}}"
```

**Parameters:**
- `cmd_type`: 1 (POSE constant)
- `position`: geometry_msgs/Point with x, y, z (meters)
- `rpy`: Roll, Pitch, Yaw (radians)

#### Move to XYZ Position Only (POSITION Mode)

**CLI Syntax:**
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 2, position: {x: 0.3, y: 0.0, z: 0.3}}}"
```

**Parameters:**
- `cmd_type`: 2 (POSITION constant)
- `position`: geometry_msgs/Point with x, y, z

#### Move to RPY Orientation Only (RPY Mode)

**CLI Syntax:**
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 3, rpy: {roll: 0.0, pitch: 1.57, yaw: 0.0}}}"
```

**Parameters:**
- `cmd_type`: 3 (RPY constant)
- `rpy`: Roll, Pitch, Yaw (radians)

#### Move to Pose with Quaternion (POSE_QUAT Mode)

**CLI Syntax:**
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 4, position: {x: 0.3, y: 0.0, z: 0.3}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"
```

**Parameters:**
- `cmd_type`: 4 (POSE_QUAT constant)
- `position`: geometry_msgs/Point
- `orientation`: geometry_msgs/Quaternion (x, y, z, w)

#### Linear Pose Move (LINEAR_POSE Mode)

**CLI Syntax:**
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 5, position: {x: 0.3, y: 0.0, z: 0.3}, rpy: {roll: 0.0, pitch: 1.57, yaw: 0.0}}}"
```

**Parameters:**
- `cmd_type`: 5 (LINEAR_POSE constant)
- `position`: geometry_msgs/Point
- `rpy`: Roll, Pitch, Yaw

#### Shift Pose Move (SHIFT_POSE Mode)

**CLI Syntax:**
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 6, shift: {axis_number: 0, value: 0.1}}}"
```

**Parameters:**
- `cmd_type`: 6 (SHIFT_POSE constant)
- `shift`: ShiftPose with axis_number (0-5) and value in meters/radians

#### All ArmMoveCommand cmd_type Constants:

```
0  = JOINTS              # uses joints array
1  = POSE               # uses position and rpy
2  = POSITION           # uses position only
3  = RPY                # uses rpy only
4  = POSE_QUAT          # uses position and quaternion orientation
5  = LINEAR_POSE        # uses position and rpy (linear motion)
6  = SHIFT_POSE         # uses shift
7  = SHIFT_LINEAR_POSE  # uses shift (linear)
8  = EXECUTE_TRAJ       # uses dist_smoothing and list_poses
9  = DRAW_SPIRAL        # special trajectory
10 = DRAW_CIRCLE        # special trajectory
11 = EXECUTE_FULL_TRAJ  # full trajectory execution
12 = EXECUTE_RAW_TRAJ   # raw trajectory execution
```

---

## 2. FOLLOW JOINT TRAJECTORY ACTION (Low-Level Control)

**Action Name:** `/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory`
**ROS2 Type:** `control_msgs/action/FollowJointTrajectory`

### Execute Joint Trajectory

**CLI Syntax:**
```bash
ros2 action send_goal /niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], \
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 0, nanosec: 0}}]}}"
```

**Parameters:**
- `trajectory`: JointTrajectory message
  - `joint_names`: List of 6 joint names in order
  - `points`: Array of JointTrajectoryPoint
    - `positions`: Array of 6 floats (joint angles in radians)
    - `velocities`: Optional velocity values
    - `accelerations`: Optional acceleration values
    - `effort`: Optional effort values
    - `time_from_start`: Duration for this waypoint

**Multi-Point Trajectory Example:**
```bash
ros2 action send_goal /niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
      {positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 0, nanosec: 0}},
      {positions: [0.5, 0.5, 0.5, 0.0, 0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}},
      {positions: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0], time_from_start: {sec: 4, nanosec: 0}}
    ]
  }}"
```

---

## 3. PYTHON EXAMPLES

### 3.1 Using Python with ROS2 Client Libraries

**Install Dependencies:**
```bash
pip install rclpy
```

**Basic Joint Move Example:**

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from niryo_ned_ros2_interfaces.action import RobotMove
from niryo_ned_ros2_interfaces.msg import ArmMoveCommand

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.client = ActionClient(self, RobotMove, '/niryo_robot_arm_commander/robot_action')
        
    def send_joint_goal(self, joints):
        """Send robot to specific joint positions"""
        goal = RobotMove.Goal()
        goal.cmd.cmd_type = 0  # JOINTS
        goal.cmd.joints = joints
        
        self.client.wait_for_server()
        self.send_goal_future = self.client.send_goal_async(goal)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Result: {result.result.message}')

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    # Move to home position
    controller.send_joint_goal([0.0, 0.57, 1.57, 0.0, 0.0, 0.0])
    
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Pose Move Example:**

```python
from geometry_msgs.msg import Point
from niryo_ned_ros2_interfaces.msg import RPY

def send_pose_goal(self, x, y, z, roll, pitch, yaw):
    """Send robot to specific TCP pose"""
    goal = RobotMove.Goal()
    goal.cmd.cmd_type = 1  # POSE
    goal.cmd.position = Point(x=x, y=y, z=z)
    goal.cmd.rpy = RPY(roll=roll, pitch=pitch, yaw=yaw)
    
    self.client.wait_for_server()
    self.client.send_goal_async(goal)
```

**FollowJointTrajectory Example:**

```python
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

def send_trajectory_goal(self, joint_values_list, time_points):
    """Execute joint trajectory with multiple waypoints"""
    client = ActionClient(
        self, 
        FollowJointTrajectory, 
        '/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory'
    )
    
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = [
        'joint_1', 'joint_2', 'joint_3', 
        'joint_4', 'joint_5', 'joint_6'
    ]
    
    for joints, duration_sec in zip(joint_values_list, time_points):
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        goal.trajectory.points.append(point)
    
    client.wait_for_server()
    client.send_goal_async(goal)
```

---

## 4. SERVICE CALLS

### 4.1 Get Forward Kinematics (Joint to Pose)

**Service Name:** `/niryo_robot/kinematics/forward`
**Type:** `niryo_ned_ros2_interfaces/srv/GetFK`

```bash
ros2 service call /niryo_robot/kinematics/forward \
  niryo_ned_ros2_interfaces/srv/GetFK \
  "{joints: [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]}"
```

### 4.2 Get Inverse Kinematics (Pose to Joint)

**Service Name:** `/niryo_robot/kinematics/inverse`
**Type:** `niryo_ned_ros2_interfaces/srv/GetIK`

```bash
ros2 service call /niryo_robot/kinematics/inverse \
  niryo_ned_ros2_interfaces/srv/GetIK \
  "{position: {x: 0.3, y: 0.0, z: 0.3}, rpy: {roll: 0.0, pitch: 1.57, yaw: 0.0}}"
```

### 4.3 Calibrate Motors

**Service Name:** `/niryo_robot/joints_interface/calibrate_motors`
**Type:** `niryo_ned_ros2_interfaces/srv/SetInt`

```bash
ros2 service call /niryo_robot/joints_interface/calibrate_motors \
  niryo_ned_ros2_interfaces/srv/SetInt "{value: 1}"
```

---

## 5. TOPICS (For Monitoring)

### Current Joint States Topic
**Topic Name:** `/joint_states`
**Type:** `sensor_msgs/msg/JointState`

```bash
ros2 topic echo /joint_states
```

### Current Robot State (TCP Pose)
**Topic Name:** `/niryo_robot/robot_state`
**Type:** `niryo_ned_ros2_interfaces/msg/RobotState`

```bash
ros2 topic echo /niryo_robot/robot_state
```

---

## 6. QUICK REFERENCE TABLE

| Command Type | Method | Action/Service | CLI Example |
|---|---|---|---|
| Move to Joints | Action | RobotMove (cmd_type: 0) | `ros2 action send_goal /niryo_robot_arm_commander/robot_action ... "{cmd: {cmd_type: 0, joints: [0,0,0,0,0,0]}}"` |
| Move to Pose | Action | RobotMove (cmd_type: 1) | `ros2 action send_goal /niryo_robot_arm_commander/robot_action ... "{cmd: {cmd_type: 1, position: {...}, rpy: {...}}}"` |
| Move to Position | Action | RobotMove (cmd_type: 2) | `ros2 action send_goal /niryo_robot_arm_commander/robot_action ... "{cmd: {cmd_type: 2, position: {...}}}"` |
| Linear Pose Move | Action | RobotMove (cmd_type: 5) | `ros2 action send_goal /niryo_robot_arm_commander/robot_action ... "{cmd: {cmd_type: 5, position: {...}, rpy: {...}}}"` |
| Trajectory Execute | Action | FollowJointTrajectory | `ros2 action send_goal /niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory ... "{trajectory: {...}}"` |
| Get FK | Service | GetFK | `ros2 service call /niryo_robot/kinematics/forward ... "{joints: [...]}"` |
| Get IK | Service | GetIK | `ros2 service call /niryo_robot/kinematics/inverse ... "{position: {...}, rpy: {...}}"` |
| Calibrate | Service | SetInt | `ros2 service call /niryo_robot/joints_interface/calibrate_motors ... "{value: 1}"` |

---

## 7. NAMESPACE SUPPORT

All commands support robot namespaces. If your robot is launched with namespace "robot1", use:

```bash
ros2 action send_goal /robot1/niryo_robot_arm_commander/robot_action ...
ros2 service call /robot1/niryo_robot/kinematics/forward ...
```

---

## 8. ERROR HANDLING

RobotMove action results include:
- `status`: Integer status code
- `message`: String with error/success message

FollowJointTrajectory action provides feedback during execution.

---

## Key Files Referenced

- **Repository:** `https://github.com/NiryoRobotics/ned-ros2-driver`
- **Message Definitions:** `/niryo_ned_ros2_interfaces/msg/ArmMoveCommand.msg`
- **Action Definitions:** `/niryo_ned_ros2_interfaces/action/RobotMove.action`
- **Official Documentation:** `https://niryorobotics.github.io/ned_ros/`

