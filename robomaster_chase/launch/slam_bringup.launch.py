"""Bringup shared by SLAM (M2) and Nav2 (M3): robot model TF + lidar frame.

Publishes:
  - the robot's URDF TF tree via robot_state_publisher (reusing
    robomaster_description's model.launch.py), prefixed with rm_name.
  - a static base_link -> laser_link transform for the simulated 2D lidar.

It deliberately does NOT launch any driver controller — the caller picks how
the robot moves (keyboard / avoiding_goto) separately.

The lidar is mounted on BaseLinkFrame at (0, 0, 0.15) in Coppelia, with its
scan plane horizontal and forward-aligned. BaseLinkFrame corresponds to the
ROS base_link, so the static transform is a pure 0.15 m z-offset.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rm_name = LaunchConfiguration("rm_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # robot_state_publisher from the description package, namespaced + prefixed.
    model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robomaster_description"), "launch", "model.launch.py",
            ])
        ),
        launch_arguments={"name": rm_name}.items(),
    )

    # Static base_link -> laser_link. The lidar sits 0.15 m above base_link,
    # centered, no rotation (scan plane already horizontal and forward in sim).
    # Frame names are prefixed to match the bridge / URDF (rm0/base_link, etc.).
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_link_tf",
        namespace=rm_name,
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.15",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", [rm_name, "/base_link"],
            "--child-frame-id", [rm_name, "/laser_link"],
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("rm_name", default_value="rm0"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Wall-clock by default (sim time breaks the bridge SDK).",
        ),
        model_launch,
        laser_tf,
    ])
