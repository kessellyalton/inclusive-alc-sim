from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_pkg = get_package_share_directory("alc_simulation")
    sim_launch = os.path.join(sim_pkg, "launch", "sim.launch.py")

    # Launch configuration
    disability_profile = LaunchConfiguration("disability_profile")
    policy_mode = LaunchConfiguration("policy_mode")
    seed = LaunchConfiguration("seed")

    # Launch arguments
    disability_profile_arg = DeclareLaunchArgument(
        "disability_profile",
        default_value="dyslexia",
        description="Disability profile to simulate (e.g., dyslexia, adhd, autism, hearing, motor, none)"
    )

    policy_mode_arg = DeclareLaunchArgument(
        "policy_mode",
        default_value="inclusive_adaptive",
        description="Policy mode: fixed, adaptive, or inclusive_adaptive"
    )

    seed_arg = DeclareLaunchArgument(
        "seed",
        default_value="42",
        description="Random seed for reproducibility"
    )

    # Include Gazebo + bridge (pass disability_profile and seed to sim launch)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "disability_profile": disability_profile,
            "seed": seed
        }.items()
    )

    # Disability profile source of truth
    disability = Node(
        package="alc_core",
        executable="disability_sim_node",
        name="disability_sim_node",
        parameters=[{
            "profile": disability_profile,
            "publish_hz": 1.0,
            "use_sim_time": True
        }],
        output="screen"
    )

    # Policy decision-making (use tutor_policy_node)
    policy = Node(
        package="alc_core",
        executable="tutor_policy_node",
        name="tutor_policy_node",
        parameters=[{
            "mode": policy_mode,
            "use_sim_time": True
        }],
        output="screen"
    )

    # Tutor interface
    interface = Node(
        package="alc_core",
        executable="tutor_interface_node",
        name="tutor_interface_node",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    # Logger
    logger = Node(
        package="alc_core",
        executable="logger_node",
        name="logger_node",
        parameters=[{
            "output_dir": os.path.join(os.path.expanduser("~"), "alc_logs"),
            "run_name": "",
            "condition": policy_mode,
            "disability_profile": disability_profile,
            "seed": seed,  # Matches learner_model_node seed for reproducibility
            "use_sim_time": True
        }],
        output="screen"
    )

    return LaunchDescription([
        disability_profile_arg,
        policy_mode_arg,
        seed_arg,
        sim,
        disability,
        policy,
        interface,
        logger
    ])
