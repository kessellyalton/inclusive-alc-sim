from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Bridges core Gazebo simulation topics to ROS 2.

    This includes:
    - /clock for simulation time
    - /tf and /tf_static for transforms
    """

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            # Simulation clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen"
    )

    return LaunchDescription([bridge])
