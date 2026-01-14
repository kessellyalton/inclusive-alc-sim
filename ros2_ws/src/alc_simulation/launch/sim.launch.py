import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    sim_pkg = get_package_share_directory("alc_simulation")
    desc_pkg = get_package_share_directory("alc_description")

    default_world = os.path.join(sim_pkg, "worlds", "classroom.sdf")
    bridge_launch = os.path.join(sim_pkg, "launch", "bridge.launch.py")

    world = LaunchConfiguration("world")
    disability_profile = LaunchConfiguration("disability_profile")
    publish_hz = LaunchConfiguration("publish_hz")
    seed = LaunchConfiguration("seed")

    declare_args = [
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("disability_profile", default_value="dyslexia"),
        DeclareLaunchArgument("publish_hz", default_value="2.0"),
        DeclareLaunchArgument("seed", default_value="42"),
    ]

    gz = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen"
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch)
    )

    # Process URDF/Xacro
    urdf_file = os.path.join(desc_pkg, "urdf", "alc_robot.urdf.xacro")
    urdf_string = xacro.process_file(urdf_file).toxml()

    # Publish robot description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": urdf_string,
            "use_sim_time": True
        }],
        output="screen"
    )

    # Spawn robot into Gazebo
    spawn_robot = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-topic", "/robot_description",
            "-name", "alc_robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5"
        ],
        output="screen"
    )

    learner = Node(
        package="alc_core",
        executable="learner_model_node",
        name="learner_model_node",
        parameters=[{
            "disability_profile": disability_profile,
            "seed": seed,
            "use_sim_time": True
        }],
        output="screen"
    )

    # Delay spawn until Gazebo is ready
    spawn_delayed = TimerAction(period=3.0, actions=[robot_state_publisher, spawn_robot])
    # Ensure /clock exists before nodes rely on sim time
    learner_delayed = TimerAction(period=2.0, actions=[learner])

    return LaunchDescription(declare_args + [gz, bridge, spawn_delayed, learner_delayed])
