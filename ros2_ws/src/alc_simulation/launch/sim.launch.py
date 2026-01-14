import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory("alc_simulation")

    default_world = os.path.join(sim_pkg, "worlds", "classroom.sdf")
    bridge_launch = os.path.join(sim_pkg, "launch", "bridge.launch.py")

    world = LaunchConfiguration("world")
    disability_profile = LaunchConfiguration("disability_profile")
    publish_hz = LaunchConfiguration("publish_hz")

    declare_args = [
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("disability_profile", default_value="dyslexia"),
        DeclareLaunchArgument("publish_hz", default_value="2.0"),
    ]

    gz = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen"
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch)
    )

    learner = Node(
        package="alc_core",
        executable="learner_model_node",
        name="learner_model_node",
        parameters=[{
            "publish_hz": publish_hz,
            "disability_profile": disability_profile,
            "use_sim_time": True  # Boolean for ROS 2 parameter
        }],
        output="screen"
    )

    # Ensure /clock exists before nodes rely on sim time
    learner_delayed = TimerAction(period=2.0, actions=[learner])

    return LaunchDescription(declare_args + [gz, bridge, learner_delayed])
