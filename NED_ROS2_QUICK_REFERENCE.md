# NED ROS2 Driver - Quick Reference Guide

**Version:** Based on ned-ros2-driver for NED2/NED3pro
**Format:** Quick reference (no tutorials) - for fast lookup

---

## 1. TOPICS (For Publishing/Subscribing)

### Core Robot State Topics

| Topic Name | Type | Direction | Description |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Publish | Current joint angles, velocities (6 joints) |
| `/niryo_robot/robot_state` | `niryo_ned_ros2_interfaces/msg/RobotState` | Publish | TCP pose (position + RPY + quaternion + twist + speed) |
| `/niryo_robot_status/robot_status` | `niryo_ned_ros2_interfaces/msg/RobotStatus` | Publish | Robot status codes, calibration state, safety flags |
| `/niryo_robot_hardware_interface/hardware_status` | `niryo_ned_ros2_interfaces/msg/HardwareStatus` | Publish | Motor temps, voltages, errors; RPi temp; calibration status |

### Transformations (TF2)

| Topic Name | Type | Direction | Description |
|---|---|---|---|
| `/tf` | `tf2_msgs/msg/TFMessage` | Publish | Dynamic transformations (joints, TCP) |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Publish | Static transformations (base, links) |

### I/O Status Topics

| Topic Name | Type | Direction | Description |
|---|---|---|---|
| `/niryo_robot_rpi/digital_io_state` | `niryo_ned_ros2_interfaces/msg/DigitalIOState` | Publish | Digital inputs/outputs state |
| `/niryo_robot_rpi/analog_io_state` | `niryo_ned_ros2_interfaces/msg/AnalogIOState` | Publish | Analog I/O state |

### Tool/Gripper Status

| Topic Name | Type | Direction | Description |
|---|---|---|---|
| `/niryo_robot_tools_commander/current_tool` | `niryo_ned_ros2_interfaces/msg/Tool` | Publish | Current tool info (ID, type, state, position) |
| `/niryo_robot_tools_commander/tool_feedback` | `niryo_ned_ros2_interfaces/msg/Tool` | Publish | Tool feedback (gripper position, vacuum state) |

### Vision/Conveyor Topics

| Topic Name | Type | Direction | Description |
|---|---|---|---|
| `/niryo_robot_vision/camera/compressed` | `sensor_msgs/msg/CompressedImage` | Publish | Camera feed (compressed) |
| `/niryo_robot_conveyor/feedback` | `niryo_ned_ros2_interfaces/msg/ConveyorFeedbackArray` | Publish | Connected conveyor status/speed |

---

## 2. ACTIONS (For Long-Running Commands)

### Primary Movement Action

**Action Path:** `/niryo_robot_arm_commander/robot_action`
**Type:** `niryo_ned_ros2_interfaces/action/RobotMove`
**Result:** `status` (int32), `message` (string)
**Feedback:** `state` (RobotState)

**Supported Commands (cmd_type):**

| cmd_type | Name | Parameters | Use Case |
|---|---|---|---|
| 0 | JOINTS | `joints: [j1, j2, j3, j4, j5, j6]` | Move to specific joint angles (radians) |
| 1 | POSE | `position: {x, y, z}` + `rpy: {roll, pitch, yaw}` | Move to TCP pose (meters + radians) |
| 2 | POSITION | `position: {x, y, z}` | Move to XYZ only, keep orientation |
| 3 | RPY | `rpy: {roll, pitch, yaw}` | Rotate TCP orientation only |
| 4 | POSE_QUAT | `position: {x, y, z}` + `orientation: {x, y, z, w}` | Move to pose with quaternion |
| 5 | LINEAR_POSE | `position: {x, y, z}` + `rpy: {roll, pitch, yaw}` | Linear cartesian motion to pose |
| 6 | SHIFT_POSE | `shift: {axis_number: 0-5, value: float}` | Relative movement per joint |
| 7 | SHIFT_LINEAR_POSE | `shift: {axis_number: 0-5, value: float}` | Linear relative movement |
| 8 | EXECUTE_TRAJ | `list_poses: [...]` + `dist_smoothing: float` | Execute waypoint trajectory |
| 9 | DRAW_SPIRAL | Special spiral parameters | Draw spiral pattern |
| 10 | DRAW_CIRCLE | Special circle parameters | Draw circle pattern |
| 11 | EXECUTE_FULL_TRAJ | `trajectory: JointTrajectory` | Execute pre-built trajectory |
| 12 | EXECUTE_RAW_TRAJ | `trajectory: JointTrajectory` | Execute raw trajectory |

### Tool Action

**Action Path:** `/niryo_robot_tools_commander/action_server`
**Type:** `niryo_ned_ros2_interfaces/action/Tool`
**Result:** `status` (int32), `message` (string)
**Feedback:** `progression` (int32, 0-100)

**Tool Commands:**

| cmd_type | Name | Parameters | Description |
|---|---|---|---|
| 1 | OPEN_GRIPPER | `tool_id`, `max_torque_percentage`, `hold_torque_percentage` | Open gripper |
| 2 | CLOSE_GRIPPER | `tool_id`, `max_torque_percentage`, `hold_torque_percentage` | Close gripper |
| 10 | PULL_AIR_VACUUM_PUMP | `tool_id`, `activate: true` | Vacuum pump pull (ON) |
| 11 | PUSH_AIR_VACUUM_PUMP | `tool_id`, `activate: false` | Vacuum pump push (OFF) |
| 20 | SETUP_DIGITAL_IO | `tool_id`, `gpio: string` | Setup digital I/O tool |
| 21 | ACTIVATE_DIGITAL_IO | `tool_id` | Activate digital output |
| 22 | DEACTIVATE_DIGITAL_IO | `tool_id` | Deactivate digital output |

### Low-Level Joint Trajectory Action

**Action Path:** `/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory`
**Type:** `control_msgs/action/FollowJointTrajectory` (standard ROS2)
**Result:** Error codes
**Feedback:** State at current point

**Use:** Multi-point trajectories with timing. Requires JointTrajectory with:
- `joint_names: [joint_1, joint_2, ..., joint_6]`
- `points: [{positions: [...], time_from_start: Duration(...)}]`

### Program Execution Action

**Action Path:** `/niryo_robot_program_executor/execute_program` (if available)
**Type:** `niryo_ned_ros2_interfaces/action/ExecuteProgram`
**Result:** `status` (int16), `message` (string)
**Feedback:** `output` (string)

---

## 3. SERVICES (For Request/Response)

### Kinematics Services

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot/kinematics/forward` | GetFK | `joints: float[]` | `pose: RobotState`, `status`, `message` | Joint angles → TCP pose |
| `/niryo_robot/kinematics/inverse` | GetIK | `pose: RobotState` | `joints: float[]`, `status`, `message` | TCP pose → Joint angles |

### Pose Management Services

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot_poses_manager/manage_pose` | ManagePose | `cmd` (SAVE/DELETE), `pose: NiryoPose` | `status`, `message` | Save/delete stored poses |
| `/niryo_robot_poses_manager/get_pose` | GetPose | `name: string` | `pose: NiryoPose`, `status`, `message` | Retrieve stored pose |
| `/niryo_robot_poses_manager/get_all_poses` | GetNameDescriptionList | - | List of pose names/descriptions | Get all saved poses |

### TCP/Tool Setup

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot_tools_commander/set_tcp` | SetTCP | `position: Point`, `rpy: RPY` OR `orientation: Quat`, `tcp_version` | `status`, `message` | Define TCP offset from tool flange |
| `/niryo_robot_tools_commander/enable_tcp` | SetBool | `data: bool` | `success` | Enable/disable TCP |

### Digital I/O

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot_rpi/set_digital_io` | SetDigitalIO | `name: string`, `value: bool` | `status`, `message` | Set digital output high/low |
| `/niryo_robot_rpi/get_digital_io` | GetDigitalIO | `name: string` | `value: bool`, `status`, `message` | Read digital input |
| `/niryo_robot_rpi/set_io_mode` | SetIOMode | `io_number`, `mode` | `status`, `message` | Set I/O as input/output |
| `/niryo_robot_rpi/set_pullup` | SetPullup | `io_number`, `pullup: bool` | `status`, `message` | Enable/disable pullup resistor |

### Analog I/O

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot_rpi/set_analog_io` | SetAnalogIO | `name: string`, `value: float` | `status`, `message` | Set analog output |
| `/niryo_robot_rpi/get_analog_io` | GetAnalogIO | `name: string` | `value: float`, `status`, `message` | Read analog input |

### Motor/Joint Control

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot/joints_interface/calibrate_motors` | SetInt | `value: 1` | `status`, `message` | Calibrate motor positions (NED2 only) |
| `/niryo_robot/joints_interface/get_joint_limits` | GetJointLimits | - | `status`, `message`, joint limits | Get joint angle ranges |

### Workspace Management

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot_workspace/manage_workspace` | ManageWorkspace | `cmd` (SAVE/DELETE), `workspace: Workspace` | `status`, `message` | Create/delete workspaces |
| `/niryo_robot_workspace/get_workspace_points` | GetWorkspacePoints | `name: string` | `points: Point[]`, `status`, `message` | Get workspace corner points |
| `/niryo_robot_workspace/get_workspace_robot_poses` | GetWorkspaceRobotPoses | `name: string` | `poses: RobotState[]`, `status`, `message` | Get workspace corner poses |
| `/niryo_robot_workspace/get_workspace_ratio` | GetWorkspaceRatio | `position: Point` | `ratio: float`, `status`, `message` | Check if point in workspace |

### Trajectory Management

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot_trajectory/manage_trajectory` | ManageTrajectory | `cmd` (SAVE/DELETE/EXECUTE), `trajectory: JointTrajectory` | `status`, `message` | Manage pre-recorded trajectories |
| `/niryo_robot_trajectory/compute_trajectory` | ComputeTrajectory | `start_pose`, `goal_pose` | `trajectory: JointTrajectory`, `status`, `message` | Generate trajectory between poses |

### Vision Services

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot_vision/object_detection` | ObjDetection | `obj_type: string`, `obj_color: string`, `workspace_ratio: float`, `ret_image: bool` | `obj_pose: ObjectPose`, `status`, `img: CompressedImage` | Detect object in image |
| `/niryo_robot_vision/debug_color_detection` | DebugColorDetection | Detection parameters | Debug visualization | Debug color detection |
| `/niryo_robot_vision/take_picture` | TakePicture | `path: string` | `success: bool` | Save picture to file |
| `/niryo_robot_vision/set_image_parameter` | SetImageParameter | Parameter settings | `status`, `message` | Configure camera |

### Conveyor Control

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot_conveyor/set_conveyor` | SetConveyor | `cmd` (ADD/REMOVE), `id: uint8` | `id`, `status`, `message`, `hardware_id` | Add/remove conveyor |
| `/niryo_robot_conveyor/control_conveyor` | ControlConveyor | `conveyor_id`, `state: bool`, `speed: int16`, `direction: int8` | `status`, `message` | Run conveyor |

### Dynamic Frame Management

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot_dynamics/manage_dynamic_frame` | ManageDynamicFrame | `cmd` (SAVE/DELETE), `dynamic_frame: DynamicFrame` | `status`, `message` | Create/delete reference frames |
| `/niryo_robot_dynamics/get_dynamic_frame` | GetDynamicFrame | `name: string` | `dynamic_frame: DynamicFrame`, `status`, `message` | Get frame definition |

### System & Diagnostics

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot/hardware_interface/hardware_status` | Trigger | - | `success`, `message` | Get hardware status |
| `/niryo_robot_diagnostics/run_auto_diagnosis` | RunAutoDiagnosis | - | `status`, `message` | Run system diagnostics |
| `/niryo_robot/system/get_cpu_temperature` | GetCpuTemperature | - | `temperature: float` | Get RPi CPU temp |
| `/niryo_robot/system/ping` | Ping | - | `success: bool`, `message: string` | Check connection |

### Settings/Config

| Service Path | Type | Input | Output | Description |
|---|---|---|---|---|
| `/niryo_robot/settings/get_settings` | GetSettings | - | `settings: Setting[]` | Get all settings |
| `/niryo_robot/settings/set_settings` | SetSettings | `settings: Setting[]` | `status`, `message` | Apply settings |
| `/niryo_robot/settings/get_string` | GetString | `name: string` | `value: string`, `status`, `message` | Get string setting |
| `/niryo_robot/settings/set_string` | SetString | `name: string`, `value: string` | `status`, `message` | Set string setting |
| `/niryo_robot/settings/get_int` | GetInt | `name: string` | `value: int32`, `status`, `message` | Get int setting |
| `/niryo_robot/settings/set_int` | SetInt | `name: string`, `value: int32` | `status`, `message` | Set int setting |
| `/niryo_robot/settings/get_float` | GetFloat | `name: string` | `value: float`, `status`, `message` | Get float setting |
| `/niryo_robot/settings/set_float` | SetFloat | `name: string`, `value: float` | `status`, `message` | Set float setting |

---

## 4. ROBOT STATUS REFERENCE

### RobotStatus Constants

From `/niryo_robot_status/robot_status` topic:

**Robot States:**
```
-7  UPDATE              Robot updating
-6  REBOOT              Robot rebooting
-5  SHUTDOWN            Robot shutting down
-4  FATAL_ERROR         Node crash
-3  MOTOR_ERROR         Electrical/disconnected motor
-2  COLLISION           Collision detected
-1  USER_PROGRAM_ERROR  Program error
0   UNKNOWN             Unknown state
1   BOOTING             Starting up
2   REBOOT_MOTOR        Motor rebooting
3   CALIBRATION_NEEDED  Needs calibration
4   CALIBRATION_IN_PROGRESS
5   LEARNING_MODE       Manual teaching mode
6   STANDBY             Powered, ready (MOST COMMON)
7   MOVING              Moving
8   RUNNING_AUTONOMOUS  User program running
9   RUNNING_DEBUG       Debug program running
10  PAUSE               Program paused
11  LEARNING_MODE_AUTONOMOUS  Teaching + program
12  LEARNING_TRAJECTORY
13  ESTOP               Emergency stop
```

**Log Status:**
```
-3  FATAL        Critical error
-2  ERROR        Error occurred
-1  WARN         Warning
0   NONE         All clear
```

### Checking Robot Ready

**Ready states:** Status = 6 (STANDBY)
**Service:** `/niryo_robot/system/ping` → `success: bool`

---

## 5. PYTHON CLIENT EXAMPLES

### Basic Template

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from niryo_ned_ros2_interfaces.action import RobotMove
from niryo_ned_ros2_interfaces.msg import ArmMoveCommand

class RobotClient(Node):
    def __init__(self):
        super().__init__('robot_client')
        self.action_client = ActionClient(
            self, RobotMove, 
            '/niryo_robot_arm_commander/robot_action'
        )
    
    def send_goal(self, goal):
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal)
```

### Joint Movement

```python
goal = RobotMove.Goal()
goal.cmd.cmd_type = 0  # JOINTS
goal.cmd.joints = [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]
future = client.send_goal(goal)
```

### Pose Movement

```python
from geometry_msgs.msg import Point
from niryo_ned_ros2_interfaces.msg import RPY

goal = RobotMove.Goal()
goal.cmd.cmd_type = 1  # POSE
goal.cmd.position = Point(x=0.3, y=0.0, z=0.3)
goal.cmd.rpy = RPY(roll=0.0, pitch=1.57, yaw=0.0)
future = client.send_goal(goal)
```

### Service Call (FK/IK)

```python
from rclpy.client import Client
from niryo_ned_ros2_interfaces.srv import GetFK

cli = node.create_client(GetFK, '/niryo_robot/kinematics/forward')
request = GetFK.Request()
request.joints = [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]
future = cli.call_async(request)

# Wait for result
rclpy.spin_until_future_complete(node, future)
response = future.result()
pose = response.pose
```

### Gripper Control

```python
from niryo_ned_ros2_interfaces.action import Tool
from niryo_ned_ros2_interfaces.msg import ToolCommand

goal = Tool.Goal()
goal.cmd.cmd_type = 2  # CLOSE_GRIPPER
goal.cmd.tool_id = 11  # Gripper1
goal.cmd.max_torque_percentage = 80
goal.cmd.hold_torque_percentage = 30
client.send_goal_async(goal)
```

### Subscribe to Robot State

```python
from niryo_ned_ros2_interfaces.msg import RobotState

def state_callback(msg: RobotState):
    x, y, z = msg.position.x, msg.position.y, msg.position.z
    roll, pitch, yaw = msg.rpy.roll, msg.rpy.pitch, msg.rpy.yaw
    print(f"Position: ({x:.3f}, {y:.3f}, {z:.3f})")
    print(f"Orientation: ({roll:.3f}, {pitch:.3f}, {yaw:.3f})")

node.create_subscription(RobotState, '/niryo_robot/robot_state', state_callback, 10)
```

### Subscribe to Robot Status

```python
from niryo_ned_ros2_interfaces.msg import RobotStatus

def status_callback(msg: RobotStatus):
    if msg.robot_status == 6:
        print("Robot is STANDBY (ready)")
    print(f"Status: {msg.robot_status_str}")
    print(f"Message: {msg.robot_message}")

node.create_subscription(RobotStatus, '/niryo_robot_status/robot_status', status_callback, 10)
```

---

## 6. COMMON USE CASE PATTERNS

### Pattern 1: Move Robot and Check Completion

**CLI:**
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 0, joints: [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]}}"
```

**Python:**
```python
goal = RobotMove.Goal()
goal.cmd.cmd_type = 0
goal.cmd.joints = [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]

future = action_client.send_goal_async(goal)
future.add_done_callback(goal_accepted_callback)

def get_result_callback(future):
    result = future.result().result
    print(f"Status: {result.status}, Message: {result.message}")
```

### Pattern 2: Linear Approach to Object

**CLI:**
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 5, position: {x: 0.3, y: 0.0, z: 0.3}, rpy: {roll: 0, pitch: 1.57, yaw: 0}}}"
```

**Python:**
```python
goal = RobotMove.Goal()
goal.cmd.cmd_type = 5  # LINEAR_POSE
goal.cmd.position = Point(x=0.3, y=0.0, z=0.3)
goal.cmd.rpy = RPY(roll=0.0, pitch=1.57, yaw=0.0)
action_client.send_goal_async(goal)
```

### Pattern 3: Open Gripper, Move, Close Gripper

```python
# Close gripper
tool_goal = Tool.Goal()
tool_goal.cmd.cmd_type = 2  # CLOSE
tool_goal.cmd.tool_id = 11
tool_goal.cmd.max_torque_percentage = 80
tool_client.send_goal_async(tool_goal)

# Wait, then move
rclpy.spin_once(node)
move_goal = RobotMove.Goal()
move_goal.cmd.cmd_type = 1
move_goal.cmd.position = Point(x=0.3, y=0.0, z=0.5)
move_goal.cmd.rpy = RPY(roll=0.0, pitch=1.57, yaw=0.0)
move_client.send_goal_async(move_goal)
```

### Pattern 4: Detect Object and Move to It

```python
# Get object position
cli = node.create_client(ObjDetection, '/niryo_robot_vision/object_detection')
request = ObjDetection.Request()
request.obj_type = "cube"
request.obj_color = "red"
future = cli.call_async(request)
rclpy.spin_until_future_complete(node, future)
obj_pose = future.result().obj_pose

# Move to object
goal = RobotMove.Goal()
goal.cmd.cmd_type = 1
goal.cmd.position = Point(x=obj_pose.x, y=obj_pose.y, z=obj_pose.z + 0.05)
goal.cmd.rpy = RPY(roll=obj_pose.roll, pitch=obj_pose.pitch, yaw=obj_pose.yaw)
action_client.send_goal_async(goal)
```

### Pattern 5: Save and Restore Pose

```python
# Save pose
cli = node.create_client(ManagePose, '/niryo_robot_poses_manager/manage_pose')
request = ManagePose.Request()
request.cmd = 1  # SAVE
request.pose.name = "home"
request.pose.description = "Home position"
request.pose.joints = [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]
future = cli.call_async(request)

# Later: Get pose
cli2 = node.create_client(GetPose, '/niryo_robot_poses_manager/get_pose')
request2 = GetPose.Request()
request2.name = "home"
future2 = cli2.call_async(request2)
rclpy.spin_until_future_complete(node, future2)
pose = future2.result().pose
```

### Pattern 6: Check Robot Status

```python
# Service approach
cli = node.create_client(Ping, '/niryo_robot/system/ping')
future = cli.call_async(Ping.Request())
rclpy.spin_until_future_complete(node, future)
if future.result().success:
    print("Robot is ready")

# Topic approach (continuous monitoring)
def status_callback(msg):
    ready = msg.robot_status == 6  # STANDBY
    error = msg.robot_status < 0
    return ready, error

node.create_subscription(RobotStatus, '/niryo_robot_status/robot_status', 
                         status_callback, 10)
```

### Pattern 7: Digital Output Control

```python
# Set digital output HIGH
cli = node.create_client(SetDigitalIO, '/niryo_robot_rpi/set_digital_io')
request = SetDigitalIO.Request()
request.name = "pin_1A"
request.value = True
future = cli.call_async(request)

# Get digital input state
cli2 = node.create_client(GetDigitalIO, '/niryo_robot_rpi/get_digital_io')
request2 = GetDigitalIO.Request()
request2.name = "pin_2A"
future2 = cli2.call_async(request2)
rclpy.spin_until_future_complete(node, future2)
state = future2.result().value
```

### Pattern 8: Inverse Kinematics

```python
cli = node.create_client(GetIK, '/niryo_robot/kinematics/inverse')
request = GetIK.Request()
request.pose.position = Point(x=0.3, y=0.0, z=0.3)
request.pose.rpy = RPY(roll=0.0, pitch=1.57, yaw=0.0)
future = cli.call_async(request)
rclpy.spin_until_future_complete(node, future)
joints = future.result().joints
```

---

## 7. KEY MESSAGE TYPES

### RobotState
```
geometry_msgs/Point position          # (x, y, z) in meters
niryo_ned_ros2_interfaces/RPY rpy     # (roll, pitch, yaw) in radians
geometry_msgs/Quaternion orientation  # (x, y, z, w)
geometry_msgs/Twist twist             # Linear + angular velocity
float64 tcp_speed                     # TCP speed in m/s
```

### RPY
```
float64 roll                          # Around X axis
float64 pitch                         # Around Y axis
float64 yaw                           # Around Z axis
```

### ArmMoveCommand (for RobotMove action)
```
int32 cmd_type                        # Movement type (0-12)
float64[] joints                      # For JOINTS mode
geometry_msgs/Point position          # XYZ position
niryo_ned_ros2_interfaces/RPY rpy     # Orientation
geometry_msgs/Quaternion orientation  # Alternative orientation format
niryo_ned_ros2_interfaces/ShiftPose shift  # For relative motion
```

### Tool
```
int8 id                               # Tool ID (11=Gripper1, 12=Gripper2, 31=Vacuum)
int8 motor_type                       # Motor type (NO_MOTOR=0, STEPPER=1, XL430=2, etc)
int16 position                        # Current position
int8 state                            # Current state
```

### ToolCommand
```
uint8 cmd_type                        # Command (OPEN=1, CLOSE=2, PULL=10, PUSH=11)
int8 tool_id                          # Target tool ID
uint16 speed                          # For gripper (Ned1/One)
uint8 max_torque_percentage           # For gripper (Ned2)
uint8 hold_torque_percentage          # For gripper (Ned2)
bool activate                         # For vacuum/electromagnet
string gpio                           # For digital I/O tools
```

### RobotStatus
```
int8 robot_status                     # Status code (-7 to 13)
string robot_status_str               # Human-readable status
string robot_message                  # Status message
int8 logs_status                      # Log level (-3 to 0)
string logs_status_str                # Log level name
string logs_message                   # Log message
bool out_of_bounds                    # Position out of workspace
bool rpi_overheating                  # Temperature warning
```

### HardwareStatus
```
int32 rpi_temperature                 # Raspberry Pi temperature (°C)
string hardware_version               # Robot model
int8 hardware_state                   # State code
bool connection_up                    # Motor bus connected
string error_message                  # Last error
bool calibration_needed               # Needs calibration
bool calibration_in_progress          # Calibrating now
string[] motor_names                  # Joint names
string[] motor_types                  # Motor types per joint
int32[] temperatures                  # Motor temperatures (°C)
float64[] voltages                    # Motor voltages
int32[] hardware_errors               # Error codes per motor
string[] hardware_errors_message      # Error messages
```

---

## 8. COORDINATE SYSTEMS & UNITS

### Position
- **X, Y, Z:** Meters (m)
- **Reference:** Robot base frame
- **TCP:** Tool Center Point (offset from tool flange)

### Orientation
- **Roll, Pitch, Yaw:** Radians (rad), range [-π, π]
  - Roll (φ): Rotation around X-axis
  - Pitch (θ): Rotation around Y-axis
  - Yaw (ψ): Rotation around Z-axis

### Joints
- **All 6 joints:** Radians (rad)
- **Range:** Typically [-π, π] (depends on model)
- **Order:** joint_1, joint_2, joint_3, joint_4, joint_5, joint_6

### Time
- **Trajectories:** `Duration(sec=int, nanosec=int)`
- `sec`: Seconds (0+)
- `nanosec`: Nanoseconds (0-999999999)

### Temperature
- Degrees Celsius (°C)

### Voltage
- Volts (V)

### Speed
- TCP speed: m/s
- Conveyor speed: % or RPM (context-dependent)

---

## 9. NAMESPACE SUPPORT

All interfaces support robot namespaces. If launched with namespace "robot1":

```bash
/robot1/niryo_robot_arm_commander/robot_action
/robot1/niryo_robot/kinematics/forward
/robot1/niryo_robot_status/robot_status
/robot1/joint_states
```

To use namespaced robot in Python:
```python
# Replace base path with namespaced path
'/robot1/niryo_robot_arm_commander/robot_action'
```

---

## 10. ERROR CODES & STATUS

### Common Status Codes (from service responses)
```
-1   Error
0    Success / OK
1    Information
-2   Communication error
-3   Hardware error
-4   Software error
```

### Common Response Fields
```
int32 status          # Status code
string message        # Detailed message
```

Check `status == 0` for success in services. For actions, check `result.status` after completion.

---

## 11. CONNECTION & DIAGNOSTICS

### Verify Connection
```bash
ros2 service call /niryo_robot/system/ping niryo_ned_ros2_interfaces/srv/Ping
```

### List Available Topics
```bash
ros2 topic list
ros2 topic echo /niryo_robot/robot_state
```

### List Available Services
```bash
ros2 service list
ros2 service call /niryo_robot/system/get_cpu_temperature \
  niryo_ned_ros2_interfaces/srv/GetCpuTemperature
```

### List Available Actions
```bash
ros2 action list
```

### Check Robot Status
```bash
ros2 topic echo /niryo_robot_status/robot_status
```

---

## 12. QUICK COMMAND REFERENCE

### Move to Home
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 0, joints: [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]}}"
```

### Move to XYZ Position
```bash
ros2 action send_goal /niryo_robot_arm_commander/robot_action \
  niryo_ned_ros2_interfaces/action/RobotMove \
  "{cmd: {cmd_type: 2, position: {x: 0.3, y: 0.0, z: 0.3}}}"
```

### Close Gripper
```bash
ros2 action send_goal /niryo_robot_tools_commander/action_server \
  niryo_ned_ros2_interfaces/action/Tool \
  "{cmd: {cmd_type: 2, tool_id: 11, max_torque_percentage: 80, hold_torque_percentage: 30}}"
```

### Get Forward Kinematics
```bash
ros2 service call /niryo_robot/kinematics/forward \
  niryo_ned_ros2_interfaces/srv/GetFK \
  "{joints: [0.0, 0.57, 1.57, 0.0, 0.0, 0.0]}"
```

### Set Digital Output
```bash
ros2 service call /niryo_robot_rpi/set_digital_io \
  niryo_ned_ros2_interfaces/srv/SetDigitalIO \
  "{name: 'pin_1A', value: true}"
```

### Monitor Joint States
```bash
ros2 topic echo /joint_states
```

### Monitor Robot State
```bash
ros2 topic echo /niryo_robot/robot_state
```

### Monitor Robot Status
```bash
ros2 topic echo /niryo_robot_status/robot_status
```

---

## NOTES

- **All angles in radians** (not degrees)
- **All distances in meters** (not cm)
- **Timestamp format:** Use ROS2 Time (built into messages)
- **Action feedback:** Available during execution (subscribe to feedback)
- **TCP transform:** Set once and use in subsequent commands
- **Workspaces:** Define once, use for collision checking
- **Trajectories:** Pre-compute or use MoveIt2 for motion planning
- **Vision:** Requires camera calibration for accurate detection
- **I/O naming:** Check robot config for exact pin names (pin_1A, pin_2B, etc.)

---

**Last Updated:** Based on ned-ros2-driver latest
**Source:** https://github.com/NiryoRobotics/ned-ros2-driver
