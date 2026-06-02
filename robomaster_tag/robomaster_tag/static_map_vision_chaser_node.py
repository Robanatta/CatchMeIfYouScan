import math
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster

from robomaster_tag.static_map_chaser_base import StaticMapChaserBase


COPPELIA_CLIENT_PATH = (
    "/Applications/coppeliaSim.app/Contents/Resources/"
    "programming/zmqRemoteApi/clients/python"
)


class StaticMapVisionChaserNode(StaticMapChaserBase):
    def __init__(self):
        super().__init__(
            node_name="static_map_vision_chaser_node",
            default_map_resolution=0.10,
            default_initial_x=-1.6749999999999994,
            default_initial_y=0.1,
            default_initial_yaw=0.0,
        )
        # Scene tuning: safer path traversal and smoother visual chase
        self.target_drive_max_error = max(self.target_drive_max_error, 0.75)
        self.static_odom_origin = None
        self.static_map_anchor = None
        self.latest_odom_pose = None
        self.coppelia_pose_active = False
        self.coppelia_pose_logged = False
        self.coppelia_pose_client = None
        self.coppelia_sim = None
        self.coppelia_pose_handle = None
        self.map_tf_broadcaster = TransformBroadcaster(self)
        self.front_safety_lookahead = max(self.front_safety_lookahead, 1.30)
        self.front_safety_width = max(self.front_safety_width, 0.95)
        self.waypoint_speed = min(self.waypoint_speed, 0.14)
        self.direct_chase_speed = min(self.direct_chase_speed, 0.16)

        self.declare_parameter("use_coppelia_pose", True)
        self.declare_parameter("coppelia_host", "127.0.0.1")
        self.declare_parameter("coppelia_port", 23000)
        self.declare_parameter(
            "coppelia_pose_object",
            f"/{self.robot_name}/reference",
        )

        self.use_coppelia_pose = bool(self.get_parameter("use_coppelia_pose").value)

        if self.use_coppelia_pose:
            self.setup_coppelia_pose()
            self.coppelia_pose_timer = self.create_timer(
                0.10,
                self.update_coppelia_pose,
                callback_group=self.timer_callback_group,
            )

    def setup_coppelia_pose(self):
        # Prefer the simulator pose because it matches the static map exactly
        for path in (COPPELIA_CLIENT_PATH, os.path.join(COPPELIA_CLIENT_PATH, "src")):
            if os.path.isdir(path) and path not in sys.path:
                sys.path.insert(0, path)

        try:
            try:
                from coppeliasim_zmqremoteapi_client import RemoteAPIClient
            except ImportError:
                from zmqRemoteApi import RemoteAPIClient

            host = str(self.get_parameter("coppelia_host").value)
            port = int(self.get_parameter("coppelia_port").value)
            pose_object = str(self.get_parameter("coppelia_pose_object").value)

            client = RemoteAPIClient(host=host, port=port)
            client.initialTimeout = 1
            sim = client.getObject("sim")
            handle = sim.getObject(pose_object)
            client.socket.RCVTIMEO = 250

            self.coppelia_pose_client = client
            self.coppelia_sim = sim
            self.coppelia_pose_handle = handle
            self.get_logger().info(
                f"Using CoppeliaSim global pose from {pose_object}."
            )
            self.update_coppelia_pose()
        except Exception as exc:
            self.get_logger().warn(
                "Could not connect to CoppeliaSim global pose. "
                f"Falling back to /odom. Reason: {exc}"
            )

    def yaw_from_quaternion(self, orientation):
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def odom_pose_from_msg(self, msg):
        position = msg.pose.pose.position
        yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        return position.x, position.y, yaw

    def map_pose_from_odom(self, odom_pose):
        # Convert local odometry motion into the map frame
        origin_x, origin_y, origin_yaw = self.static_odom_origin
        anchor_x, anchor_y, anchor_yaw = self.static_map_anchor
        odom_x, odom_y, odom_yaw = odom_pose

        local_x = odom_x - origin_x
        local_y = odom_y - origin_y
        c = math.cos(anchor_yaw)
        s = math.sin(anchor_yaw)

        map_x = anchor_x + c * local_x - s * local_y
        map_y = anchor_y + s * local_x + c * local_y
        map_yaw = self.normalize_angle(anchor_yaw + odom_yaw - origin_yaw)

        return map_x, map_y, map_yaw

    def map_to_odom_transform(self, map_pose, odom_pose):
        # RViz needs this transform to draw map, path, and robot together
        map_x, map_y, map_yaw = map_pose
        odom_x, odom_y, odom_yaw = odom_pose
        yaw = self.normalize_angle(map_yaw - odom_yaw)
        c = math.cos(yaw)
        s = math.sin(yaw)
        x = map_x - (c * odom_x - s * odom_y)
        y = map_y - (s * odom_x + c * odom_y)

        return x, y, yaw

    def update_coppelia_pose(self):
        if self.coppelia_sim is None or self.coppelia_pose_handle is None:
            return

        # Read the robot reference frame directly from the running scene
        try:
            position = self.coppelia_sim.getObjectPosition(
                self.coppelia_pose_handle,
                self.coppelia_sim.handle_world,
            )
            orientation = self.coppelia_sim.getObjectOrientation(
                self.coppelia_pose_handle,
                self.coppelia_sim.handle_world,
            )
        except Exception as exc:
            self.get_logger().warn(
                "Lost CoppeliaSim global pose. Falling back to /odom. "
                f"Reason: {exc}"
            )
            self.coppelia_pose_active = False
            self.coppelia_sim = None
            self.coppelia_pose_handle = None
            return

        self.pose = (
            float(position[0]),
            float(position[1]),
            self.normalize_angle(float(orientation[2])),
        )
        self.has_odom = True
        self.coppelia_pose_active = True
        self.last_dead_reckoning_time = 0.0

        if not self.coppelia_pose_logged:
            self.get_logger().info(
                "CoppeliaSim pose locked at "
                f"x={self.pose[0]:.2f}, y={self.pose[1]:.2f}, yaw={self.pose[2]:.2f}."
            )
            self.coppelia_pose_logged = True

    def odom_callback(self, msg):
        odom_pose = self.odom_pose_from_msg(msg)
        self.latest_odom_pose = odom_pose

        # CoppeliaSim pose is the map truth; odometry is only the fallback
        if self.coppelia_pose_active:
            return

        if self.static_odom_origin is None:
            # First odom message fixes the fallback map-to-odom offset
            anchor = self.pose or (self.initial_x, self.initial_y, self.initial_yaw)
            self.static_odom_origin = odom_pose
            self.static_map_anchor = anchor
            self.get_logger().info(
                "Static map odometry anchored at "
                f"map x={anchor[0]:.2f}, y={anchor[1]:.2f}, "
                f"yaw={anchor[2]:.2f}; "
                f"odom x={odom_pose[0]:.2f}, y={odom_pose[1]:.2f}, yaw={odom_pose[2]:.2f}."
            )

        self.has_odom = True
        self.pose = self.map_pose_from_odom(odom_pose)

    def publish_static_map_transform(self):
        transform_pose = (self.initial_x, self.initial_y, self.initial_yaw)

        latest_odom_pose = getattr(self, "latest_odom_pose", None)
        static_odom_origin = getattr(self, "static_odom_origin", None)
        static_map_anchor = getattr(self, "static_map_anchor", None)

        if self.pose is not None and latest_odom_pose is not None:
            transform_pose = self.map_to_odom_transform(self.pose, latest_odom_pose)
        elif static_odom_origin is not None and static_map_anchor is not None:
            transform_pose = self.map_to_odom_transform(
                static_map_anchor,
                static_odom_origin,
            )

        transform_x, transform_y, transform_yaw = transform_pose
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = transform_x
        transform.transform.translation.y = transform_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(0.5 * transform_yaw)
        transform.transform.rotation.w = math.cos(0.5 * transform_yaw)

        broadcaster = getattr(
            self,
            "map_tf_broadcaster",
            self.static_tf_broadcaster,
        )
        broadcaster.sendTransform(transform)

    def publish_planning_grid(self):
        msg = self.make_occupancy_grid_message(self.display_grid)
        self.planning_map_pub.publish(msg)

    def should_replan(self, goal=None):
        if goal is None:
            goal = self.last_target_world

        if self.pose is None or goal is None:
            return False

        if not self.path:
            return True

        if self.path_goal is None:
            return True

        if self.path_map_revision != self.map_revision:
            return True

        moved_goal = math.hypot(
            goal[0] - self.path_goal[0],
            goal[1] - self.path_goal[1],
        )

        return moved_goal > 0.35

    def path_exists_to_goal(self, goal):
        # Exploration should choose only goals that the planner can reach
        if self.pose is None or goal is None:
            return False

        start = self.world_to_grid(self.pose[0], self.pose[1])
        goal_cell = self.world_to_grid(goal[0], goal[1])

        if start is None or goal_cell is None:
            return False

        blocked_grid = self.occupancy_grid

        start = self.planning_cell(start, blocked_grid, "start", 0.8, log=False)
        goal_cell = self.planning_cell(goal_cell, blocked_grid, "goal", 2.0, log=False)

        if start is None or goal_cell is None:
            return False

        return bool(self.astar(start, goal_cell, blocked_grid))

    def choose_next_search_goal(self):
        for _ in range(max(1, self.random_goal_attempts)):
            goal = self.random_free_goal()

            if goal is None:
                continue

            if self.path_exists_to_goal(goal):
                self.random_goal_count += 1
                return goal

        return None

    def follow_path_command(self):
        if self.pose is None or not self.path:
            return None

        # Turn first, then drive forward so the camera stays in front
        robot_x, robot_y, robot_yaw = self.pose

        while self.path:
            waypoint_x, waypoint_y = self.path[0]
            distance = math.hypot(waypoint_x - robot_x, waypoint_y - robot_y)

            if distance >= 0.25:
                break

            self.path.pop(0)

        if not self.path:
            return None

        waypoint_x, waypoint_y = self.path[0]
        dx = waypoint_x - robot_x
        dy = waypoint_y - robot_y
        distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - robot_yaw)

        cmd = Twist()
        cmd.angular.z = self.clamp(1.1 * yaw_error, -0.40, 0.40)

        if abs(yaw_error) < 0.22:
            heading_scale = self.clamp(1.0 - abs(yaw_error) / 0.22, 0.25, 1.0)
            distance_scale = self.clamp(distance / 0.60, 0.35, 1.0)
            cmd.linear.x = self.waypoint_speed * heading_scale * distance_scale

        return cmd

    def direct_chase_command(self, area, normalized_error):
        # When the runner is visible, chase it directly
        cmd = Twist()

        if abs(normalized_error) > self.target_center_deadband:
            cmd.angular.z = self.clamp(-0.7 * normalized_error, -0.45, 0.45)

        if area >= 3000:
            return cmd

        error = min(abs(normalized_error), 1.0)
        heading_scale = self.clamp(1.0 - error, 0.20, 1.0)
        distance_scale = self.clamp((3000.0 - area) / 2500.0, 0.35, 1.0)
        cmd.linear.x = self.direct_chase_speed * heading_scale * distance_scale

        return cmd


def main(args=None):
    rclpy.init(args=args)

    node = StaticMapVisionChaserNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
