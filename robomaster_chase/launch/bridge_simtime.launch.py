"""Wrapper around the upstream robomaster_ros bridge that forces use_sim_time.

The bridge's ep.launch / main.launch (an upstream git submodule we don't want
to edit) don't expose use_sim_time, so its TF gets stamped with wall-clock
time. That clashes with Coppelia's sim-time scan + /clock, and slam_toolbox
drops every scan ("timestamp earlier than all data in the transform cache").

launch_ros's SetParameter sets use_sim_time on ALL nodes spawned within this
launch scope — including the bridge node included below — so its get_clock()
switches to /clock (sim time) and its TF stamps line up with the scan.

Usage (replaces the plain `ros2 launch robomaster_ros ep.launch name:=rm0`):
    ros2 launch robomaster_chase bridge_simtime.launch.py rm_name:=rm0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rm_name = LaunchConfiguration("rm_name")

    bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robomaster_ros"), "launch", "ep.launch",
            ])
        ),
        launch_arguments={"name": rm_name}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rm_name", default_value="rm0"),
        # Force sim time on every node in this scope, including the bridge.
        SetParameter(name="use_sim_time", value=True),
        bridge,
    ])
