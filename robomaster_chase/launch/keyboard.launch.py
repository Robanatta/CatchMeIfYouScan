from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rm_name", default_value="rm0"),
            Node(
                package="robomaster_chase",
                executable="keyboard_controller",
                namespace=LaunchConfiguration("rm_name"),
                output="screen",
                # Keyboard input needs a real TTY, so don't detach stdin/stdout.
                emulate_tty=True,
                prefix="",
            ),
        ]
    )
