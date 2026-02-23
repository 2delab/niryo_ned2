#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle
import time
import math

from niryo_ned_ros2_interfaces.action import RobotMove, Tool
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
)
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class MockActionServer(Node):
    def __init__(self):
        super().__init__("mock_action_server")

        # Action servers
        self._robot_action_server = ActionServer(
            self,
            RobotMove,
            "/niryo_robot_arm_commander/robot_action",
            execute_callback=self.handle_robot_move,
        )

        self._tool_action_server = ActionServer(
            self,
            Tool,
            "/niryo_robot_tools_commander/action_server",
            execute_callback=self.handle_tool,
        )

        # MoveIt planning service client
        self.plan_service = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        while not self.plan_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /plan_kinematic_path service...")

        # Arm controller action client
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        while not self.arm_action_client.server_is_ready():
            self.get_logger().info(
                "Waiting for /arm_controller/follow_joint_trajectory action..."
            )
            time.sleep(0.1)

        # Joint state tracking
        self.current_joint_state = None
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        # Wait for initial joint state
        while self.current_joint_state is None:
            self.get_logger().info("Waiting for /joint_states...")
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Mock action servers started")

    def _joint_state_callback(self, msg: JointState):
        """Track current joint state"""
        self.current_joint_state = msg

    def handle_robot_move(self, goal_handle: ServerGoalHandle):
        """Handle RobotMove action - cmd_type 0-5"""
        try:
            cmd = goal_handle.request.cmd
            self.get_logger().info(f"RobotMove: cmd_type={cmd.cmd_type}")

            # Build constraints based on cmd_type
            if cmd.cmd_type == 0:  # JOINTS
                constraints = self._build_joint_constraints(cmd.joints)
            elif cmd.cmd_type == 1:  # POSE (position + rpy)
                constraints = self._build_pose_constraints(cmd.position, cmd.rpy)
            elif cmd.cmd_type == 4:  # POSE_QUAT (position + quaternion)
                constraints = self._build_pose_quat_constraints(
                    cmd.position, cmd.orientation
                )
            elif cmd.cmd_type == 5:  # LINEAR_POSE (position + rpy)
                constraints = self._build_pose_constraints(cmd.position, cmd.rpy)
            elif cmd.cmd_type == 2:  # POSITION only
                constraints = self._build_position_constraints(cmd.position)
            elif cmd.cmd_type == 3:  # RPY only
                constraints = self._build_rpy_constraints(cmd.rpy)
            else:
                result = RobotMove.Result(
                    status=-2, message=f"Unsupported cmd_type: {cmd.cmd_type}"
                )
                goal_handle.abort()
                return result

            # Plan trajectory
            trajectory = self._plan_trajectory(constraints)
            if trajectory is None:
                result = RobotMove.Result(status=-1, message="Motion planning failed")
                goal_handle.abort()
                return result

            # Execute trajectory
            self._execute_trajectory(trajectory)

            result = RobotMove.Result(status=0, message="ok")
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"Error in robot move: {str(e)}")
            result = RobotMove.Result(status=-1, message=f"Error: {str(e)}")
            goal_handle.abort()
            return result

    def handle_tool(self, goal_handle: ServerGoalHandle):
        """Handle Tool action - gripper control"""
        try:
            cmd = goal_handle.request.cmd
            if cmd.cmd_type == 1:
                self.get_logger().info("Gripper: OPEN")
            elif cmd.cmd_type == 2:
                self.get_logger().info("Gripper: CLOSE")
            else:
                self.get_logger().info(f"Tool command: cmd_type={cmd.cmd_type}")

            # Simulate execution
            time.sleep(0.5)

            result = Tool.Result(status=0, message="ok")
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"Error in tool action: {str(e)}")
            result = Tool.Result(status=-1, message=f"Error: {str(e)}")
            goal_handle.abort()
            return result

    def _build_joint_constraints(self, joints):
        """Build joint constraints from joint values"""
        constraints = Constraints()
        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

        for i, name in enumerate(joint_names):
            if i < len(joints):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = joints[i]
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

        return constraints

    def _build_pose_constraints(self, position, rpy):
        """Build position + orientation constraints from position and RPY"""
        constraints = Constraints()

        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = "world"
        pc.link_name = "tool_link"
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        pc.constraint_region.primitives = []
        pc.constraint_region.primitive_poses = []
        pc.weight = 1.0

        # Create bounding box constraint
        from shape_msgs.msg import SolidPrimitive

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = [0.01, 0.01, 0.01]
        pc.constraint_region.primitives.append(sp)

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position = position
        pc.constraint_region.primitive_poses.append(pose.pose)
        constraints.position_constraints.append(pc)

        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = "world"
        oc.link_name = "tool_link"
        quat = self._rpy_to_quaternion(rpy.roll, rpy.pitch, rpy.yaw)
        oc.orientation.x = quat[0]
        oc.orientation.y = quat[1]
        oc.orientation.z = quat[2]
        oc.orientation.w = quat[3]
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        return constraints

    def _build_pose_quat_constraints(self, position, orientation):
        """Build position + orientation constraints from position and quaternion"""
        constraints = Constraints()

        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = "world"
        pc.link_name = "tool_link"
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        pc.weight = 1.0

        from shape_msgs.msg import SolidPrimitive

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = [0.01, 0.01, 0.01]
        pc.constraint_region.primitives.append(sp)

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position = position
        pc.constraint_region.primitive_poses.append(pose.pose)
        constraints.position_constraints.append(pc)

        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = "world"
        oc.link_name = "tool_link"
        oc.orientation = orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        return constraints

    def _build_position_constraints(self, position):
        """Build position constraint only"""
        constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = "world"
        pc.link_name = "tool_link"
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        pc.weight = 1.0

        from shape_msgs.msg import SolidPrimitive

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = [0.01, 0.01, 0.01]
        pc.constraint_region.primitives.append(sp)

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position = position
        pc.constraint_region.primitive_poses.append(pose.pose)
        constraints.position_constraints.append(pc)

        return constraints

    def _build_rpy_constraints(self, rpy):
        """Build orientation constraint only"""
        constraints = Constraints()

        oc = OrientationConstraint()
        oc.header.frame_id = "world"
        oc.link_name = "tool_link"
        quat = self._rpy_to_quaternion(rpy.roll, rpy.pitch, rpy.yaw)
        oc.orientation.x = quat[0]
        oc.orientation.y = quat[1]
        oc.orientation.z = quat[2]
        oc.orientation.w = quat[3]
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        return constraints

    def _plan_trajectory(self, goal_constraints):
        """Plan trajectory using MoveIt"""
        try:
            request = GetMotionPlan.Request()
            request.motion_plan_request.group_name = "arm"
            request.motion_plan_request.num_planning_attempts = 3
            request.motion_plan_request.allowed_planning_time = 5.0
            request.motion_plan_request.goal_constraints.append(goal_constraints)

            future = self.plan_service.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result() is None:
                return None

            result = future.result()
            if not result.motion_plan_response.trajectory.joint_trajectory.points:
                return None

            return result.motion_plan_response.trajectory.joint_trajectory

        except Exception as e:
            self.get_logger().error(f"Planning failed: {str(e)}")
            return None

    def _execute_trajectory(self, trajectory):
        """Execute trajectory via arm controller action"""
        try:
            trajectory.header.stamp = self.get_clock().now().to_msg()

            # Send trajectory via arm controller action
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory

            future = self.arm_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result() is None:
                self.get_logger().error("Arm controller action failed")
                return False

            self.get_logger().info("Trajectory executed")
            return True

        except Exception as e:
            self.get_logger().error(f"Execution failed: {str(e)}")
            return False

    @staticmethod
    def _rpy_to_quaternion(roll, pitch, yaw):
        """Convert RPY to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return (qx, qy, qz, qw)


def main():
    rclpy.init()
    node = MockActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
