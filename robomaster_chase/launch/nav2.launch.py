"""Nav2 navigation for the RoboMaster EP (Lidar + SLAM + Nav2).

Global path planning around walls + assertive path following, replacing the
reactive avoidance. slam_toolbox (localization mode) loads the saved map and is
the single publisher of /map (latched) and map -> rm0/odom; Nav2's global
costmap loads /map, NavFn plans a route around obstacles, and
RegulatedPurePursuit drives /rm0/cmd_vel.

Pipeline:
  slam_bringup            -> robot URDF TF + base_link->laser_link static TF
  scan_restamp            -> wall-clock /rm0/scan_stamped (RELIABLE)
  async_slam_toolbox_node -> /map + map -> rm0/odom, in either mode:
                               slam_mode:=localization (default) loads a saved
                                 map -- behind-a-wall planning works at once;
                               slam_mode:=mapping builds the map live while
                                 navigating -- no pre-map, but the global
                                 planner only routes around walls already seen.
  Nav2 lifecycle servers  -> controller / planner / smoother / behaviors /
                             bt_navigator / waypoint_follower
  lifecycle_manager       -> autostart, brings them all to ACTIVE

Nav2 nodes run GLOBAL (no namespace); frame params carry the rm0/ prefix
(see config/nav2_params.yaml) and topics are absolute. cmd_vel is remapped to
/rm0/cmd_vel on the final emitters.

Prereqs (everything wall-clock; sim time breaks the bridge SDK):
  - CoppeliaSim playing, scene with the working sweeping 270deg lidar
  - ros2 launch robomaster_ros ep.launch name:=rm0   (wait for "connected")
  - for localization mode only: a saved map at maps/<map_name>.{posegraph,data}
    (default: room). Mapping mode needs no pre-saved map.

Run (autonomous tour of a waypoint list, default):
  ros2 launch robomaster_chase nav2.launch.py rm_name:=rm0 \
      waypoints:="[1.0, 1.0, -1.0, 1.0, -1.0, -1.0]"
  # Robot A drives the points by itself; Nav2 plans around walls + dodges
  # obstacles per leg. Add loop:=true to repeat forever, slam_mode:=mapping to
  # build the map live instead of loading the saved one.

Drive MANUALLY instead (no auto tour; click goals in RViz):
  ros2 launch robomaster_chase nav2.launch.py rm_name:=rm0 auto_send:=false
  # then in RViz (Fixed Frame map) use "2D Goal Pose". A goal BEHIND A WALL is
  # routed around immediately in localization mode.

Single map->odom owner: do NOT also run ground_truth_localizer, the mapping
slam_toolbox, AMCL, or the old goto/avoiding controllers (they fight over
map->rm0/odom or /rm0/cmd_vel).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory


def _slam_node(context, *args, **kwargs):
    """Build the slam_toolbox node based on slam_mode (mapping | localization).

    Done in an OpaqueFunction because the config file and whether to pass
    map_file_name depend on the resolved arg value:
      - mapping:      slam_toolbox.yaml,  build the map live (blank start)
      - localization: slam_toolbox_localization.yaml, load maps/<map_name>
    Either way slam_toolbox is the single publisher of /map + map -> rm0/odom.
    """
    mode = LaunchConfiguration("slam_mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    map_name = LaunchConfiguration("map_name").perform(context)
    share = get_package_share_directory("robomaster_chase")

    if mode == "mapping":
        cfg = os.path.join(share, "config", "slam_toolbox.yaml")
        params = [cfg, {"use_sim_time": use_sim_time}]
    else:  # localization
        cfg = os.path.join(share, "config", "slam_toolbox_localization.yaml")
        params = [cfg, {
            "use_sim_time": use_sim_time,
            "map_file_name": os.path.join(share, "maps", map_name),
        }]

    return [Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=params,
    )]


def generate_launch_description():
    rm_name = LaunchConfiguration("rm_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    open_rviz = LaunchConfiguration("rviz")

    pkg = FindPackageShare("robomaster_chase")
    nav2_params = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    rviz_config = PathJoinSubstitution([pkg, "config", "slam.rviz"])

    # cmd_vel chain: controller -> cmd_vel_nav -> velocity_smoother -> /rm0/cmd_vel.
    cmd_vel_topic = [rm_name, "/cmd_vel"]

    # 1. Shared TF bringup: robot URDF tree + base_link->laser_link static TF.
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "slam_bringup.launch.py"])
        ),
        launch_arguments={"rm_name": rm_name, "use_sim_time": use_sim_time}.items(),
    )

    # 2. Wall-clock scan -> /rm0/scan_stamped (RELIABLE), consumed by the
    #    costmaps and slam_toolbox.
    scan_restamp = Node(
        package="robomaster_chase",
        executable="scan_restamp",
        name="scan_restamp",
        namespace=rm_name,
        output="screen",
        parameters=[{"input_topic": "scan", "output_topic": "scan_stamped"}],
    )

    # 3. SINGLE map->odom owner: slam_toolbox. mapping (build live) or
    #    localization (load saved map) per the slam_mode arg. Built in an
    #    OpaqueFunction (see _slam_node) since the config + map_file_name depend
    #    on the resolved mode. Publishes /map latched + map -> rm0/odom either way.
    slam = OpaqueFunction(function=_slam_node)

    # 4. Nav2 lifecycle servers (GLOBAL; frame params carry rm0/ in the yaml).
    #    Node names MUST match the yaml top-level keys AND the lifecycle
    #    node_names list below.
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )
    smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params],
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),          # input from controller
            ("cmd_vel_smoothed", cmd_vel_topic),  # output to the bridge
        ],
    )
    behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
        # Recovery behaviors (spin/backup) publish cmd_vel directly, bypassing
        # the smoother -> remap straight to the bridge.
        remappings=[("cmd_vel", cmd_vel_topic)],
    )
    bt_nav = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params],
    )
    waypoint = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params],
    )

    # 5. Lifecycle manager: brings the servers to ACTIVE in order. node_names
    #    must exactly match the Node name= strings above.
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": [
                "controller_server",
                "planner_server",
                "velocity_smoother",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
            ],
        }],
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

    # 6. Autonomous waypoint tour: hands the waypoint list to Nav2's
    #    waypoint_follower so Robot A drives the points by itself (Nav2 plans
    #    around walls + dodges obstacles per leg). Gated by auto_send so you can
    #    disable it and drive via RViz 2D Goal Pose instead.
    waypoint_sender = Node(
        package="robomaster_chase",
        executable="nav2_waypoint_sender",
        name="nav2_waypoint_sender",
        output="screen",
        parameters=[{
            "waypoints": LaunchConfiguration("waypoints"),
            "loop": LaunchConfiguration("loop"),
            "frame_id": "map",
        }],
        condition=IfCondition(LaunchConfiguration("auto_send")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rm_name", default_value="rm0"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Wall-clock by default (sim time breaks the bridge SDK).",
        ),
        DeclareLaunchArgument(
            "slam_mode", default_value="localization",
            description="localization = load the saved map (behind-a-wall "
                        "planning works immediately); mapping = build the map "
                        "live while navigating (no pre-map needed, but global "
                        "planning to unseen areas is unreliable until scanned).",
        ),
        DeclareLaunchArgument(
            "map_name", default_value="room",
            description="Saved map basename (no extension) under the package "
                        "maps/ dir. Used only in localization mode.",
        ),
        DeclareLaunchArgument(
            "auto_send", default_value="true",
            description="Auto-send the waypoint list to Nav2 so Robot A tours "
                        "the points autonomously. Set false to drive manually "
                        "via RViz 2D Goal Pose.",
        ),
        DeclareLaunchArgument(
            "waypoints", default_value="[1.0, 0.0]",
            description="Flat list [x0,y0,x1,y1,...] of MAP-frame points for the "
                        "autonomous tour (auto_send:=true).",
        ),
        DeclareLaunchArgument(
            "loop", default_value="false",
            description="Repeat the waypoint tour forever (default: visit once).",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open RViz with the map/costmap view.",
        ),
        bringup,
        scan_restamp,
        slam,
        controller,
        planner,
        smoother,
        behaviors,
        bt_nav,
        waypoint,
        lifecycle,
        rviz,
        waypoint_sender,
    ])
