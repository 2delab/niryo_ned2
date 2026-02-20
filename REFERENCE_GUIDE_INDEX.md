# NED ROS2 Driver - Reference Guide Index

## Overview

Complete structured reference documentation for Niryo NED2/NED3pro ROS2 interfaces.

---

## Quick Start Files

### 1. **INTERFACE_SUMMARY.txt** (This file)
- High-level overview of all interfaces
- Quick reference to topics, actions, services
- Robot status codes
- Common patterns
- Perfect for getting oriented

### 2. **NED_ROS2_QUICK_REFERENCE.md** (789 lines)
- Complete detailed reference
- Full interface tables with syntax
- Python client examples
- Message type definitions
- Units and coordinate systems
- All use case patterns

---

## Organization by Use Case

### Robot Movement
**Files:**
- `NED_ROS2_QUICK_REFERENCE.md` - Section 2 (Actions)
- `MOVEMENT_COMMANDS_GUIDE.md` - Comprehensive movement guide
- `MOVEMENT_COMMANDS_INDEX.md` - Quick index

**Key Interfaces:**
```
Action:   /niryo_robot_arm_commander/robot_action
Service:  /niryo_robot/kinematics/forward
Service:  /niryo_robot/kinematics/inverse
```

**Most Used Commands:**
- `cmd_type 0` - Move to joint angles
- `cmd_type 1` - Move to TCP pose (XYZ + RPY)
- `cmd_type 5` - Linear cartesian motion
- `cmd_type 2` - Move to XYZ only

---

### Gripper & Tool Control
**Files:**
- `NED_ROS2_QUICK_REFERENCE.md` - Section 2 (Tool Action)

**Key Interface:**
```
Action: /niryo_robot_tools_commander/action_server
```

**Commands:**
- `cmd_type 1` - Open gripper
- `cmd_type 2` - Close gripper
- `cmd_type 10` - Vacuum pull
- `cmd_type 11` - Vacuum push

---

### Robot Status & Monitoring
**Files:**
- `NED_ROS2_QUICK_REFERENCE.md` - Sections 1, 4

**Key Topics:**
```
/niryo_robot_status/robot_status        (Status codes, safety)
/niryo_robot/robot_state                (Current pose)
/joint_states                           (Joint angles)
/niryo_robot_hardware_interface/hardware_status  (Hardware info)
```

**Key Services:**
```
/niryo_robot/system/ping                (Connection check)
/niryo_robot/system/get_cpu_temperature (Get temperature)
```

---

### Vision & Object Detection
**Files:**
- `NED_ROS2_QUICK_REFERENCE.md` - Section 3 (Vision Services)

**Key Services:**
```
/niryo_robot_vision/object_detection    (Detect objects)
/niryo_robot_vision/take_picture        (Capture image)
/niryo_robot_vision/set_image_parameter (Camera config)
```

---

### Digital & Analog I/O
**Files:**
- `NED_ROS2_QUICK_REFERENCE.md` - Section 3 (I/O Services)

**Key Services:**
```
/niryo_robot_rpi/set_digital_io  (Set pin output)
/niryo_robot_rpi/get_digital_io  (Read pin input)
/niryo_robot_rpi/set_analog_io   (Set analog value)
/niryo_robot_rpi/get_analog_io   (Read analog value)
```

---

### Workspace Operations
**Files:**
- `NED_ROS2_QUICK_REFERENCE.md` - Section 3 (Workspace Services)

**Key Services:**
```
/niryo_robot_workspace/manage_workspace       (Create/delete)
/niryo_robot_workspace/get_workspace_points   (Get corners)
/niryo_robot_workspace/get_workspace_ratio    (Check collision)
```

---

## Command Lookup

### By Interface Type

#### Topics (Publish/Subscribe)
- `/joint_states` - Current joint angles
- `/niryo_robot/robot_state` - Current TCP pose
- `/niryo_robot_status/robot_status` - Robot status
- `/niryo_robot_hardware_interface/hardware_status` - Hardware info
- `/tf`, `/tf_static` - Transformations
- `/niryo_robot_rpi/digital_io_state` - Digital I/O states
- `/niryo_robot_rpi/analog_io_state` - Analog I/O states
- `/niryo_robot_vision/camera/compressed` - Camera feed
- `/niryo_robot_conveyor/feedback` - Conveyor status
- `/niryo_robot_tools_commander/current_tool` - Active tool

#### Actions (Long-running commands)
- `/niryo_robot_arm_commander/robot_action` - **PRIMARY: All robot movements**
- `/niryo_robot_tools_commander/action_server` - Gripper/tool commands
- `/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory` - Trajectory execution

#### Services (Request/Response)
- **Kinematics**: `/niryo_robot/kinematics/forward` & `inverse`
- **Poses**: `/niryo_robot_poses_manager/*`
- **TCP**: `/niryo_robot_tools_commander/set_tcp` & `enable_tcp`
- **I/O**: `/niryo_robot_rpi/set_digital_io` & `get_digital_io`
- **Workspace**: `/niryo_robot_workspace/*`
- **Vision**: `/niryo_robot_vision/object_detection`
- **Conveyor**: `/niryo_robot_conveyor/set_conveyor` & `control_conveyor`
- **System**: `/niryo_robot/system/ping`
- **Settings**: `/niryo_robot/settings/get_*` & `set_*`

---

## Robot Status Codes Quick Reference

| Code | Status | Meaning |
|------|--------|---------|
| 6 | STANDBY | **Ready to move** |
| 7 | MOVING | Executing motion |
| -1 | USER_PROGRAM_ERROR | Program failed |
| -2 | COLLISION | Collision detected |
| -3 | MOTOR_ERROR | Motor issue |
| -4 | FATAL_ERROR | Critical error |

**Check ready:** `robot_status == 6`

---

## Python Client Patterns

### 1. Move to Joints
```python
goal = RobotMove.Goal()
goal.cmd.cmd_type = 0
goal.cmd.joints = [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]
client.send_goal_async(goal)
```

### 2. Move to Pose
```python
goal = RobotMove.Goal()
goal.cmd.cmd_type = 1
goal.cmd.position = Point(x=0.3, y=0.0, z=0.3)
goal.cmd.rpy = RPY(roll=0.0, pitch=1.57, yaw=0.0)
client.send_goal_async(goal)
```

### 3. Close Gripper
```python
goal = Tool.Goal()
goal.cmd.cmd_type = 2
goal.cmd.tool_id = 11
goal.cmd.max_torque_percentage = 80
client.send_goal_async(goal)
```

### 4. Get Forward Kinematics
```python
request = GetFK.Request()
request.joints = [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]
response = client.call_async(request)
```

### 5. Monitor Status
```python
def callback(msg):
    if msg.robot_status == 6:
        print("Robot ready")
node.create_subscription(RobotStatus, '/niryo_robot_status/robot_status', callback, 10)
```

### 6. Digital Output Control
```python
request = SetDigitalIO.Request()
request.name = "pin_1A"
request.value = True
client.call_async(request)
```

---

## Units & Conventions

| Quantity | Unit | Range |
|----------|------|-------|
| Position (X, Y, Z) | Meters (m) | Robot workspace |
| Rotation (Roll, Pitch, Yaw) | Radians (rad) | [-π, π] |
| Joint Angles | Radians (rad) | [-π, π] |
| Temperature | Celsius (°C) | |
| Voltage | Volts (V) | |
| TCP Speed | m/s | |

---

## Namespace Support

All interfaces support namespaces. If robot launched with namespace "robot1":

```bash
/robot1/niryo_robot_arm_commander/robot_action
/robot1/niryo_robot/kinematics/forward
/robot1/niryo_robot_status/robot_status
```

---

## File Locations

### Documentation Files
- Main guide: `/home/i/niryo_ned2/NED_ROS2_QUICK_REFERENCE.md`
- Movement guide: `/home/i/niryo_ned2/MOVEMENT_COMMANDS_GUIDE.md`
- Movement index: `/home/i/niryo_ned2/MOVEMENT_COMMANDS_INDEX.md`
- This index: `/home/i/niryo_ned2/REFERENCE_GUIDE_INDEX.md`

### Source Code
- Interface definitions: `/home/i/niryo_ned2/src/ned-ros2-driver/niryo_ned_ros2_interfaces/`
- Driver implementation: `/home/i/niryo_ned2/src/ned-ros2-driver/niryo_ned_ros2_driver/`
- Tests/examples: `/home/i/niryo_ned2/src/ned-ros2-driver/niryo_ned_ros2_driver/tests/`

---

## Common Commands

### Move Home
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 0, joints: [0, 0.57, 1.57, 0, 0, 0]}}"
```

### Check Status
```bash
ros2 topic echo /niryo_robot_status/robot_status
```

### Get Position
```bash
ros2 topic echo /niryo_robot/robot_state
```

### Forward Kinematics
```bash
ros2 service call /niryo_robot/kinematics/forward \
  niryo_ned_ros2_interfaces/srv/GetFK \
  "{joints: [0, 0.57, 1.57, 0, 0, 0]}"
```

### Set Digital Output
```bash
ros2 service call /niryo_robot_rpi/set_digital_io \
  niryo_ned_ros2_interfaces/srv/SetDigitalIO \
  "{name: 'pin_1A', value: true}"
```

---

## Quick Navigation

**I want to...**

- **Move the robot to a position** → See "Robot Movement" section
- **Control the gripper** → See "Gripper & Tool Control" section
- **Check if robot is ready** → Look for robot status code 6 (STANDBY)
- **Detect objects** → See "Vision & Object Detection" section
- **Control I/O pins** → See "Digital & Analog I/O" section
- **See current pose** → Subscribe to `/niryo_robot/robot_state` topic
- **Write a Python client** → See "Python Client Patterns" section
- **Understand message formats** → See `NED_ROS2_QUICK_REFERENCE.md` Section 7

---

## Key Takeaways

1. **All movements use**: `/niryo_robot_arm_commander/robot_action` (cmd_type 0-12)
2. **Robot ready state**: Status = 6 (check via `/niryo_robot_status/robot_status`)
3. **All units**: Meters, radians, degrees Celsius
4. **All angles**: In radians, NOT degrees
5. **All services**: Return status=0 for success, check message for errors
6. **Gripper control**: Tool ID 11=Gripper1, 12=Gripper2
7. **TCP offset**: Set once via `/niryo_robot_tools_commander/set_tcp`
8. **Namespaces**: Supported, prefix all paths with `/namespace/`

---

**Document Generated:** February 20, 2025
**Source:** ned-ros2-driver repository analysis
**Format:** Quick reference - for immediate lookup
