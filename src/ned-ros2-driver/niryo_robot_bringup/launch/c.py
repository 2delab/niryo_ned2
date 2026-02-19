from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ==================== CONFIGURATIONS ====================
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    gazebo_verbosity = LaunchConfiguration("gazebo_verbosity", default="4")
    gazebo_world = LaunchConfiguration(
        "gazebo_world",
        default="/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf",
    )

    # Package paths
    niryo_description_share = FindPackageShare("niryo_ned_description")
    niryo_controllers_share = FindPackageShare("niryo_ned2_gripper_moveit_config")

    # ==================== ENVIRONMENT SETUP ====================
    set_gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        PathJoinSubstitution([niryo_description_share, ".."]),
    )

    suppress_logging = SetEnvironmentVariable("RCL_LOGGING_CONFIG_DIR", "")

    # ==================== ROBOT DESCRIPTION ====================
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    niryo_description_share,
                    "urdf",
                    "ned2",
                    "niryo_ned2_gazebo.urdf.xacro",
                ]
            ),
            " controllers_config_file:=",
            PathJoinSubstitution(
                [niryo_controllers_share, "config", "ros2_controllers.yaml"]
            ),
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ==================== NODES ====================
    # Clock Bridge (Gazebo → ROS2) - FIXED: [ = subscribe from Gazebo
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        name="clock_bridge",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        name="robot_state_publisher",
    )

    # ROS2 Control Node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [niryo_controllers_share, "config", "ros2_controllers.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        name="ros2_control_node",
    )

    # Spawn Entity in Gazebo (delayed)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "niryo_ned2",
            "-allow_renaming",
            "true",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        name="gazebo_spawner",
    )

    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity])

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        name="joint_state_broadcaster_spawner",
    )

    delayed_broadcaster = TimerAction(period=1.0, actions=[joint_state_broadcaster])

    # Joint Trajectory Controller
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "-c",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        name="joint_trajectory_controller_spawner",
    )

    delayed_trajectory = TimerAction(period=1.0, actions=[joint_trajectory_controller])

    # Gripper Controller
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        name="gripper_controller_spawner",
    )

    delayed_gripper = TimerAction(period=1.0, actions=[gripper_controller])

    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulated clock from Gazebo",
            ),
            DeclareLaunchArgument(
                "gazebo_verbosity",
                default_value="4",
                description="Gazebo verbosity level (0-4)",
            ),
            DeclareLaunchArgument(
                "gazebo_world",
                default_value="/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf",
                description="Path to Gazebo world SDF file",
            ),
            # Environment
            set_gazebo_resource_path,
            suppress_logging,
            # Gazebo (include launch)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                    )
                ),
                launch_arguments=[
                    (
                        "gz_args",
                        [
                            " -r -v ",
                            gazebo_verbosity,
                            " ",
                            gazebo_world,
                        ],
                    ),
                    ("use_sim_time", use_sim_time),
                ],
            ),
            # Clock Bridge (start immediately)
            clock_bridge,
            # Robot Publisher
            robot_state_publisher,
            # ROS2 Control Node (start immediately)
            ros2_control_node,
            # Spawn entity (delayed 5s for Gazebo + bridge startup)
            delayed_spawn_entity,
            # Event handlers for sequential startup
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[delayed_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[delayed_trajectory],
                )
            ),
            # Event handler for gripper controller
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_trajectory_controller,
                    on_exit=[delayed_gripper],
                )
            ),
            # MoveIt Move Group for motion planning
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("niryo_ned2_gripper_moveit_config"),
                            "launch",
                            "move_group.launch.py",
                        ]
                    )
                ),
                launch_arguments=[
                    ("use_sim_time", use_sim_time),
                ],
            ),
        ]
    )
