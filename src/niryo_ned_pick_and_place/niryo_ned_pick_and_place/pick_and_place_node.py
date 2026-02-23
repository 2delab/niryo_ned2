#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import yaml
import os

from niryo_ned_ros2_interfaces.action import RobotMove, Tool
from niryo_ned_ros2_interfaces.msg import ArmMoveCommand, ToolCommand, RPY
from geometry_msgs.msg import Point


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_and_place_node")

        # Load configuration
        self.declare_parameter("config_file", "")
        config_path = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )

        if not config_path:
            # Default path relative to the package
            config_path = "/home/i/niryo_ned2/src/niryo_ned_pick_and_place/config/pick_and_place.yaml"

        self.get_logger().info(f"Loading config from: {config_path}")
        try:
            with open(config_path, "r") as f:
                self.config = yaml.safe_load(f)["pick_and_place"]
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {str(e)}")
            return

        # Action Clients
        self.robot_client = ActionClient(
            self, RobotMove, "/niryo_robot_arm_commander/robot_action"
        )
        self.tool_client = ActionClient(
            self, Tool, "/niryo_robot_tools_commander/action_server"
        )

        self.get_logger().info("Waiting for action servers...")
        self.robot_client.wait_for_server()
        self.tool_client.wait_for_server()
        self.get_logger().info("Action servers ready")

    def send_robot_move(self, cmd_type, position=None, rpy=None, joints=None):
        goal_msg = RobotMove.Goal()
        goal_msg.cmd.cmd_type = cmd_type

        if position:
            goal_msg.cmd.position = Point(
                x=float(position["x"]), y=float(position["y"]), z=float(position["z"])
            )
        if rpy:
            goal_msg.cmd.rpy = RPY(
                roll=float(rpy["roll"]),
                pitch=float(rpy["pitch"]),
                yaw=float(rpy["yaw"]),
            )
        if joints:
            goal_msg.cmd.joints = [float(j) for j in joints]

        self.get_logger().info(f"Sending RobotMove: type={cmd_type}")
        future = self.robot_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.status < 0:
            self.get_logger().error(
                f"RobotMove failed with status {result.status}: {result.message}"
            )
            return False
        return True

    def send_tool_command(self, cmd_type):
        goal_msg = Tool.Goal()
        goal_msg.cmd.cmd_type = cmd_type
        goal_msg.cmd.tool_id = self.config.get("tool_id", 11)
        goal_msg.cmd.max_torque_percentage = self.config.get("gripper_torque", 50)

        self.get_logger().info(f"Sending Tool command: type={cmd_type}")
        future = self.tool_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Tool goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.status < 0:
            self.get_logger().error(
                f"Tool command failed with status {result.status}: {result.message}"
            )
            return False
        return True

    def run_task(self):
        self.get_logger().info("Starting Pick and Place Task...")

        # 1. Open Gripper
        if not self.send_tool_command(ToolCommand.OPEN_GRIPPER):
            return

        # 2. Approach Pick
        pick_pose = self.config["pick_pose"]
        approach_height = self.config["approach_height"]

        approach_pick = {
            "x": pick_pose["position"]["x"],
            "y": pick_pose["position"]["y"],
            "z": pick_pose["position"]["z"] + approach_height,
        }

        if not self.send_robot_move(
            ArmMoveCommand.POSE, approach_pick, pick_pose["rpy"]
        ):
            return

        # 3. Descend Pick (Linear)
        if not self.send_robot_move(
            ArmMoveCommand.LINEAR_POSE, pick_pose["position"], pick_pose["rpy"]
        ):
            return

        # 4. Grasp
        if not self.send_tool_command(ToolCommand.CLOSE_GRIPPER):
            return

        # 5. Lift
        if not self.send_robot_move(
            ArmMoveCommand.LINEAR_POSE, approach_pick, pick_pose["rpy"]
        ):
            return

        # 6. Approach Place
        place_pose = self.config["place_pose"]
        approach_place = {
            "x": place_pose["position"]["x"],
            "y": place_pose["position"]["y"],
            "z": place_pose["position"]["z"] + approach_height,
        }

        if not self.send_robot_move(
            ArmMoveCommand.POSE, approach_place, place_pose["rpy"]
        ):
            return

        # 7. Descend Place (Linear)
        if not self.send_robot_move(
            ArmMoveCommand.LINEAR_POSE, place_pose["position"], place_pose["rpy"]
        ):
            return

        # 8. Release
        if not self.send_tool_command(ToolCommand.OPEN_GRIPPER):
            return

        # 9. Retract
        if not self.send_robot_move(
            ArmMoveCommand.LINEAR_POSE, approach_place, place_pose["rpy"]
        ):
            return

        self.get_logger().info("Pick and Place Task Completed Successfully!")


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    if hasattr(node, "config"):
        node.run_task()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
