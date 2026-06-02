"""M2: online SLAM with slam_toolbox.

Brings up the robot TF + lidar frame (slam_bringup) and runs
async_slam_toolbox_node against /rm0/scan, producing /map and the
map -> rm0/odom transform.

Everything runs on sim time (use_sim_time:=true), so Coppelia MUST be
publishing /clock and the bridge MUST be launched with use_sim_time:=true:

    ros2 launch robomaster_ros ep.launch name:=rm0 use_sim_time:=true

This launch file does NOT start a driver — pick how the robot moves
separately (keyboard for manual mapping, or avoiding_goto for an
autonomous perimeter tour). Optionally opens RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rm_name = LaunchConfiguration("rm_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    open_rviz = LaunchConfiguration("rviz")

    pkg = FindPackageShare("robomaster_chase")
    slam_config = PathJoinSubstitution([pkg, "config", "slam_toolbox.yaml"])
    rviz_config = PathJoinSubstitution([pkg, "config", "slam.rviz"])

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "slam_bringup.launch.py"])
        ),
        launch_arguments={
            "rm_name": rm_name,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Restamp the sim-time scan onto wall-clock so it lines up with the
    # bridge's wall-clock TF. Publishes /rm0/scan_stamped.
    scan_restamp = Node(
        package="robomaster_chase",
        executable="scan_restamp",
        name="scan_restamp",
        namespace=rm_name,
        output="screen",
        parameters=[{"input_topic": "scan", "output_topic": "scan_stamped"}],
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config, {"use_sim_time": use_sim_time}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(open_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rm_name", default_value="rm0"),
        # Wall-clock everywhere (sim time breaks the bridge SDK). The scan is
        # restamped to wall-clock by scan_restamp instead.
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open RViz with the SLAM view.",
        ),
        bringup,
        scan_restamp,
        slam,
        rviz,
    ])
