import heapq
import math
import random
import time

import cv2
import numpy as np

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from cv_bridge import CvBridge


# Obstacle footprints from the CoppeliaSim scene, in map coordinates
STATIC_RECTANGLES = [
    (0.0, 6.0, 12.0, 0.18),
    (0.0, -6.0, 12.0, 0.18),
    (6.0, 0.0, 0.18, 12.0),
    (-6.0, 0.0, 0.18, 12.0),
    (-3.2, 1.8, 0.35, 4.4),
    (3.0, -1.7, 0.35, 4.7),
    (0.1, 3.35, 4.4, 0.35),
    (-0.1, -3.15, 4.0, 0.35),
    (0.0, 0.0, 1.25, 1.05),
    (-4.25, -2.3, 2.0, 0.30),
    (4.15, 2.3, 2.0, 0.30),
    (-1.9, -4.55, 0.35, 1.9),
    (1.9, 4.55, 0.35, 1.9),
]

STATIC_CIRCLES = [
    (-4.6, 4.3, 0.35),
    (4.65, -4.2, 0.35),
]

# Red wraps around hue 0, so OpenCV needs two ranges
RED_HSV_RANGES = [
    (np.array([0, 100, 100]), np.array([10, 255, 255])),
    (np.array([170, 100, 100]), np.array([180, 255, 255])),
]

NEIGHBORS_8 = [
    (-1, -1), (0, -1), (1, -1),
    (-1, 0),           (1, 0),
    (-1, 1),  (0, 1),  (1, 1),
]


class StaticMapChaserBase(Node):
    def __init__(
        self,
        node_name="static_map_chaser_base",
        default_map_resolution=0.20,
        default_initial_x=-4.7,
        default_initial_y=-4.8,
        default_initial_yaw=0.0,
    ):
        super().__init__(node_name)

        # Shared ROS state, map state, and controller settings
        self.bridge = CvBridge()
        self.last_state = None
        self.last_search_turn = 1.0
        self.image_callback_group = MutuallyExclusiveCallbackGroup()
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        defaults = {
            "robot_name": "rm1",
            "map_size": 12.0,
            "map_resolution": default_map_resolution,
            "robot_radius": 0.35,
            "obstacle_padding": 0.20,
            "camera_fov": 1.05,
            "lost_turn_duration": 2.0,
            "scan_duration": 8.0,
            "waypoint_speed": 0.18,
            "front_safety_width": 0.70,
            "front_safety_lookahead": 0.95,
            "front_safety_step": 0.20,
            "direct_chase_speed": 0.20,
            "target_center_deadband": 0.06,
            "target_drive_deadband": 0.14,
            "target_drive_max_error": 0.55,
            "map_border_keepout": 0.85,
            "random_goal_min_distance": 1.0,
            "random_goal_attempts": 30,
            "map_frame": "map",
            "use_dead_reckoning_if_no_odom": True,
            "initial_x": default_initial_x,
            "initial_y": default_initial_y,
            "initial_yaw": default_initial_yaw,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.robot_name = self.get_parameter("robot_name").value.strip("/")
        self.odom_frame = f"{self.robot_name}/odom" if self.robot_name else "odom"
        self.map_frame = self.get_parameter("map_frame").value

        float_params = [
            "map_size",
            "map_resolution",
            "robot_radius",
            "obstacle_padding",
            "camera_fov",
            "lost_turn_duration",
            "scan_duration",
            "waypoint_speed",
            "front_safety_width",
            "front_safety_lookahead",
            "front_safety_step",
            "direct_chase_speed",
            "target_center_deadband",
            "target_drive_deadband",
            "target_drive_max_error",
            "map_border_keepout",
            "random_goal_min_distance",
            "initial_x",
            "initial_y",
            "initial_yaw",
        ]
        for name in float_params:
            setattr(self, name, float(self.get_parameter(name).value))

        self.random_goal_attempts = int(self.get_parameter("random_goal_attempts").value)
        self.use_dead_reckoning_if_no_odom = bool(
            self.get_parameter("use_dead_reckoning_if_no_odom").value
        )

        self.grid_width = int(round(self.map_size / self.map_resolution))
        self.grid_height = self.grid_width
        self.origin_x = -0.5 * self.map_size
        self.origin_y = -0.5 * self.map_size
        self.occupancy_grid = np.zeros(
            (self.grid_height, self.grid_width),
            dtype=np.uint8,
        )
        self.display_grid = np.zeros(
            (self.grid_height, self.grid_width),
            dtype=np.uint8,
        )

        self.has_odom = False
        self.pose = None
        if self.use_dead_reckoning_if_no_odom:
            # This lets the robot start moving even before odometry arrives
            self.pose = (self.initial_x, self.initial_y, self.initial_yaw)
            self.get_logger().info(
                "Odometry fallback enabled. Starting internal pose estimate at "
                f"x={self.initial_x:.2f}, y={self.initial_y:.2f}, yaw={self.initial_yaw:.2f}."
            )
        self.last_cmd = Twist()
        self.last_dead_reckoning_time = time.monotonic()
        self.last_target_world = None
        self.last_seen_time = None
        self.target_lost_time = None
        self.map_revision = 0
        self.path = []
        self.path_goal = None
        self.path_map_revision = -1
        self.last_plan_time = 0.0
        self.scan_start_time = None
        self.scan_last_yaw = None
        self.scan_turn_total = 0.0
        self.finished_last_seen_scan = False
        self.current_search_goal = None
        self.random_goal_count = 0

        # The static scene is known, so planning can start immediately
        self.load_static_big_scene_map()

        self.image_sub = self.create_subscription(
            Image,
            f"/{self.robot_name}/camera/image_color",
            self.image_callback,
            10,
            callback_group=self.image_callback_group,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            f"/{self.robot_name}/cmd_vel",
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            f"/{self.robot_name}/debug/red_detection",
            10,
        )

        self.mask_pub = self.create_publisher(
            Image,
            f"/{self.robot_name}/debug/red_mask",
            10,
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            f"/{self.robot_name}/map",
            1,
        )

        self.planning_map_pub = self.create_publisher(
            OccupancyGrid,
            f"/{self.robot_name}/planning_map",
            1,
        )

        self.path_pub = self.create_publisher(
            Path,
            f"/{self.robot_name}/planned_path",
            1,
        )

        self.debug_timer = self.create_timer(
            0.5,
            self.publish_debug_map,
            callback_group=self.timer_callback_group,
        )
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_map_transform()

        self.odom_sub = self.create_subscription(
            Odometry,
            f"/{self.robot_name}/odom",
            self.odom_callback,
            10,
            callback_group=self.sensor_callback_group,
        )

        self.get_logger().info(
            "Using camera, pose, static map, and A* path planning."
        )

        self.get_logger().info("Vision chaser started. Looking for the red sphere.")

    def publish_static_map_transform(self):
        # Keep map and odom in the same frame unless a subclass changes it
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(transform)

    def update_dead_reckoning(self):
        # Estimate pose from the last command while odometry is unavailable
        if self.has_odom or not self.use_dead_reckoning_if_no_odom:
            return

        if self.pose is None:
            self.pose = (self.initial_x, self.initial_y, self.initial_yaw)

        now = time.monotonic()
        dt = now - self.last_dead_reckoning_time
        self.last_dead_reckoning_time = now

        if dt <= 0.0:
            return

        x, y, yaw = self.pose
        vx = self.last_cmd.linear.x
        vy = self.last_cmd.linear.y
        omega = self.last_cmd.angular.z

        x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
        y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
        yaw = self.normalize_angle(yaw + omega * dt)

        self.pose = (x, y, yaw)

    def remember_command(self, cmd):
        # Store the command so dead reckoning can integrate it later
        remembered = Twist()
        remembered.linear.x = cmd.linear.x
        remembered.linear.y = cmd.linear.y
        remembered.linear.z = cmd.linear.z
        remembered.angular.x = cmd.angular.x
        remembered.angular.y = cmd.angular.y
        remembered.angular.z = cmd.angular.z
        self.last_cmd = remembered

    def publish_command(self, cmd):
        # Update local pose before sending the next velocity command
        self.update_dead_reckoning()
        self.remember_command(cmd)
        self.cmd_pub.publish(cmd)

    def publish_debug_map(self):
        self.update_dead_reckoning()
        self.publish_static_map_transform()
        # RViz gets the real obstacle map, the inflated planning map, and the path
        self.publish_occupancy_grid()
        self.publish_planning_grid()
        self.publish_planned_path()

    def make_occupancy_grid_message(self, grid):
        # Convert a numpy grid into the ROS map message used by RViz.
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.map_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = [
            100 if int(cell) else 0
            for row in grid
            for cell in row
        ]

        return msg

    def publish_occupancy_grid(self):
        # Publish the scene map without robot-radius inflation
        msg = self.make_occupancy_grid_message(self.display_grid)
        self.map_pub.publish(msg)

    def publish_planning_grid(self):
        # Publish the inflated map used by A*
        msg = self.make_occupancy_grid_message(self.occupancy_grid)
        self.planning_map_pub.publish(msg)

    def make_path_pose(self, x, y):
        # Convert one waypoint into a Path pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.03
        pose.pose.orientation.w = 1.0

        return pose

    def publish_planned_path(self):
        # Draw the current robot pose and remaining waypoints in RViz
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        if self.pose is not None:
            msg.poses.append(self.make_path_pose(self.pose[0], self.pose[1]))

        for waypoint_x, waypoint_y in self.path:
            msg.poses.append(self.make_path_pose(waypoint_x, waypoint_y))

        self.path_pub.publish(msg)

    def set_state(self, state):
        # Log state changes once, instead of repeating every camera frame
        if state != self.last_state:
            self.get_logger().info(state)
            self.last_state = state

    def make_turn_command(self, direction=None):
        # Create a pure rotation command for search behavior
        cmd = Twist()

        if direction is None:
            direction = self.last_search_turn

        cmd.angular.z = 0.3 * direction

        return cmd

    def normalize_angle(self, angle):
        # Keep angles in the standard -pi to pi range
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def clamp(self, value, low, high):
        # Bound controller outputs
        return max(low, min(high, value))

    def clear_path(self):
        # Drop the current planned path
        self.path = []
        self.path_goal = None
        self.path_map_revision = -1

    def reset_scan(self):
        # Restart the one-turn search state
        self.scan_start_time = None
        self.scan_last_yaw = None
        self.scan_turn_total = 0.0

    def odom_callback(self, msg):
        # Use odometry directly in the base node
        if not self.has_odom:
            self.get_logger().info(
                "Received odometry. Using /odom instead of internal pose estimate."
            )

        self.has_odom = True
        self.last_dead_reckoning_time = time.monotonic()

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

        self.pose = (position.x, position.y, yaw)

    def world_to_grid(self, x, y):
        # Convert map coordinates to grid indexes
        grid_x = int((x - self.origin_x) / self.map_resolution)
        grid_y = int((y - self.origin_y) / self.map_resolution)

        if not (0 <= grid_x < self.grid_width):
            return None

        if not (0 <= grid_y < self.grid_height):
            return None

        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        # Convert grid indexes to the center of a map cell
        x = self.origin_x + (grid_x + 0.5) * self.map_resolution
        y = self.origin_y + (grid_y + 0.5) * self.map_resolution

        return x, y

    def mark_static_rectangle_on_grid(
        self,
        grid,
        center_x,
        center_y,
        size_x,
        size_y,
        inflation,
    ):
        # Mark every grid cell covered by a rectangular obstacle
        x_min = center_x - 0.5 * size_x - inflation
        x_max = center_x + 0.5 * size_x + inflation
        y_min = center_y - 0.5 * size_y - inflation
        y_max = center_y + 0.5 * size_y + inflation

        for grid_y in range(self.grid_height):
            for grid_x in range(self.grid_width):
                x, y = self.grid_to_world(grid_x, grid_y)

                if x_min <= x <= x_max and y_min <= y <= y_max:
                    grid[grid_y, grid_x] = 1

    def mark_static_circle_on_grid(self, grid, center_x, center_y, radius, inflation):
        # Mark every grid cell covered by a circular obstacle
        inflated_radius = radius + inflation

        for grid_y in range(self.grid_height):
            for grid_x in range(self.grid_width):
                x, y = self.grid_to_world(grid_x, grid_y)

                if math.hypot(x - center_x, y - center_y) <= inflated_radius:
                    grid[grid_y, grid_x] = 1

    def load_static_big_scene_map(self):
        inflation = self.robot_radius + self.obstacle_padding

        # Display grid shows the scene. Occupancy grid is inflated for the robot body
        for center_x, center_y, size_x, size_y in STATIC_RECTANGLES:
            self.mark_static_rectangle_on_grid(
                self.display_grid,
                center_x,
                center_y,
                size_x,
                size_y,
                0.0,
            )
            self.mark_static_rectangle_on_grid(
                self.occupancy_grid,
                center_x,
                center_y,
                size_x,
                size_y,
                inflation,
            )

        for center_x, center_y, radius in STATIC_CIRCLES:
            self.mark_static_circle_on_grid(
                self.display_grid,
                center_x,
                center_y,
                radius,
                0.0,
            )
            self.mark_static_circle_on_grid(
                self.occupancy_grid,
                center_x,
                center_y,
                radius,
                inflation,
            )

        occupied_cells = int(np.count_nonzero(self.occupancy_grid))
        display_cells = int(np.count_nonzero(self.display_grid))
        self.map_revision += 1
        self.get_logger().info(
            "Loaded static big-scene map with "
            f"{display_cells} scene cells and {occupied_cells} planning cells."
        )

    def astar(self, start, goal, blocked_grid):
        # Find a shortest free-cell path on the planning grid
        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}
        cost_so_far = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                break

            current_x, current_y = current

            for dx, dy in NEIGHBORS_8:
                next_x = current_x + dx
                next_y = current_y + dy

                if not (0 <= next_x < self.grid_width):
                    continue

                if not (0 <= next_y < self.grid_height):
                    continue

                if blocked_grid[next_y, next_x]:
                    continue

                if dx != 0 and dy != 0:
                    # Do not cut diagonally through obstacle corners
                    if blocked_grid[current_y, next_x] or blocked_grid[next_y, current_x]:
                        continue

                next_cell = (next_x, next_y)
                step_cost = math.hypot(dx, dy)
                new_cost = cost_so_far[current] + step_cost

                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    heuristic = math.hypot(goal[0] - next_x, goal[1] - next_y)
                    priority = new_cost + heuristic
                    heapq.heappush(open_set, (priority, next_cell))
                    came_from[next_cell] = current

        if goal not in came_from and start != goal:
            return []

        path = [goal]
        current = goal

        while current != start:
            current = came_from[current]
            path.append(current)

        path.reverse()

        return path

    def nearest_free_cell(self, desired_cell, blocked_grid, max_radius_m=1.5):
        # Search outward until a nearby safe free cell is found.
        max_radius_cells = max(1, int(math.ceil(max_radius_m / self.map_resolution)))
        desired_x, desired_y = desired_cell
        best_cell = None
        best_score = None

        for radius in range(max_radius_cells + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue

                    cell_x = desired_x + dx
                    cell_y = desired_y + dy

                    if not (0 <= cell_x < self.grid_width):
                        continue

                    if not (0 <= cell_y < self.grid_height):
                        continue

                    if blocked_grid[cell_y, cell_x]:
                        continue

                    world_x, world_y = self.grid_to_world(cell_x, cell_y)

                    if not self.inside_safe_floor(world_x, world_y):
                        continue

                    score = math.hypot(dx, dy)

                    if best_score is None or score < best_score:
                        best_score = score
                        best_cell = (cell_x, cell_y)

            if best_cell is not None:
                return best_cell

        return None

    def planning_cell(self, cell, blocked_grid, name, max_radius_m, log=True):
        if not blocked_grid[cell[1], cell[0]]:
            return cell

        # Start and goal can be inside inflated obstacles; use nearby free space
        free_cell = self.nearest_free_cell(cell, blocked_grid, max_radius_m)

        if free_cell is None:
            if log:
                self.get_logger().warn(
                    f"A* failed: {name} area is occupied and no nearby free cell "
                    f"was found: {cell}."
                )
            return None

        if log:
            self.get_logger().info(
                f"A*: {name} cell {cell} is occupied, using nearest free cell {free_cell}."
            )

        return free_cell

    def plan_path_to_target(self, goal=None):
        # Build a waypoint path from the robot to the target
        if goal is None:
            goal = self.last_target_world

        if self.pose is None or goal is None:
            return False

        start = self.world_to_grid(self.pose[0], self.pose[1])
        goal_cell = self.world_to_grid(
            goal[0],
            goal[1],
        )

        if start is None or goal_cell is None:
            self.get_logger().warn("A* failed: start or goal is outside the floor grid.")
            self.clear_path()
            return False

        blocked_grid = self.occupancy_grid.copy()

        start = self.planning_cell(start, blocked_grid, "start", 0.8)
        goal_cell = self.planning_cell(goal_cell, blocked_grid, "goal", 2.0)

        if start is None or goal_cell is None:
            self.clear_path()
            return False

        path_cells = self.astar(start, goal_cell, blocked_grid)

        if not path_cells:
            self.get_logger().warn(
                f"A* failed: no path from {start} to {goal_cell}."
            )
            self.clear_path()
            return False

        sparse_cells = path_cells[::2]

        if sparse_cells[-1] != path_cells[-1]:
            sparse_cells.append(path_cells[-1])

        self.path = [
            self.grid_to_world(grid_x, grid_y)
            for grid_x, grid_y in sparse_cells[1:]
        ]
        self.path_goal = goal
        self.path_map_revision = self.map_revision
        self.last_plan_time = time.monotonic()

        return True

    def should_replan(self, goal=None):
        # Replan only when the path is missing, stale, or the goal moved
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

        if time.monotonic() - self.last_plan_time > 1.0:
            return True

        moved_goal = math.hypot(
            goal[0] - self.path_goal[0],
            goal[1] - self.path_goal[1],
        )

        return moved_goal > 0.35

    def follow_path_command(self):
        if self.pose is None or not self.path:
            return None

        # Follow waypoints with the camera facing forward
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
        cmd.angular.z = self.clamp(1.0 * yaw_error, -0.45, 0.45)

        if abs(yaw_error) < 0.25:
            heading_scale = self.clamp(1.0 - abs(yaw_error) / 0.25, 0.35, 1.0)
            distance_scale = self.clamp(distance / 0.60, 0.35, 1.0)
            cmd.linear.x = self.waypoint_speed * heading_scale * distance_scale
        else:
            cmd.linear.x = 0.0

        cmd.linear.y = 0.0

        if self.unsafe_to_drive_forward() and cmd.linear.x > 0.02:
            self.clear_path()
            self.set_state("SAFETY: forward path blocked, stopping and replanning")
            return Twist()

        return cmd

    def line_to_goal_blocked(self, goal):
        # Check whether a straight chase line crosses the static map
        if self.pose is None or goal is None:
            return False

        robot_x, robot_y, _ = self.pose
        goal_x, goal_y = goal
        distance = math.hypot(goal_x - robot_x, goal_y - robot_y)

        if distance <= 0.0:
            return False

        step = max(0.5 * self.map_resolution, 0.05)
        start_clearance = 0.25
        goal_clearance = max(0.60, self.robot_radius + self.obstacle_padding)
        check_distance = distance - goal_clearance

        if check_distance <= start_clearance:
            return False

        steps = max(1, int(math.ceil((check_distance - start_clearance) / step)))

        for index in range(1, steps + 1):
            t = index / steps
            sample_distance = start_clearance + t * (check_distance - start_clearance)
            sample = sample_distance / distance
            x = robot_x + sample * (goal_x - robot_x)
            y = robot_y + sample * (goal_y - robot_y)

            if not self.inside_safe_floor(x, y):
                return True

            cell = self.world_to_grid(x, y)

            if cell is None:
                return True

            grid_x, grid_y = cell

            if self.occupancy_grid[grid_y, grid_x]:
                return True

        return False

    def inside_safe_floor(self, x, y):
        # Keep patrol and safety checks away from the wall border
        return (
            self.origin_x + self.map_border_keepout <= x
            <= self.origin_x + self.map_size - self.map_border_keepout
            and self.origin_y + self.map_border_keepout <= y
            <= self.origin_y + self.map_size - self.map_border_keepout
        )

    def known_map_obstacle_ahead(self):
        if self.pose is None:
            return False

        # Check a small rectangle in front of the robot before driving
        robot_x, robot_y, robot_yaw = self.pose
        half_width = max(0.5 * self.front_safety_width, self.robot_radius)
        forward_step = max(self.front_safety_step, self.map_resolution)
        lookahead = max(self.front_safety_lookahead, forward_step)
        lateral_count = max(3, int(math.ceil((2.0 * half_width) / self.map_resolution)) + 1)
        forward_count = max(1, int(math.ceil(lookahead / forward_step)))

        for forward_index in range(1, forward_count + 1):
            forward = min(forward_index * forward_step, lookahead)

            for lateral_index in range(lateral_count):
                if lateral_count == 1:
                    lateral = 0.0
                else:
                    lateral = -half_width + (
                        2.0 * half_width * lateral_index / (lateral_count - 1)
                    )

                check_x = (
                    robot_x
                    + forward * math.cos(robot_yaw)
                    - lateral * math.sin(robot_yaw)
                )
                check_y = (
                    robot_y
                    + forward * math.sin(robot_yaw)
                    + lateral * math.cos(robot_yaw)
                )
                cell = self.world_to_grid(check_x, check_y)

                if cell is None or not self.inside_safe_floor(check_x, check_y):
                    return True

                grid_x, grid_y = cell

                if self.occupancy_grid[grid_y, grid_x]:
                    return True

        return False

    def unsafe_to_drive_forward(self):
        # Stop forward motion when the map says the front is blocked
        return self.known_map_obstacle_ahead()

    def estimate_target_world_position(self, area, normalized_error):
        # Approximate where the runner is in map coordinates
        if self.pose is None:
            return None

        robot_x, robot_y, robot_yaw = self.pose
        bearing = -normalized_error * 0.5 * self.camera_fov
        distance = self.clamp(36.0 / math.sqrt(area), 0.45, 4.0)

        target_x = robot_x + distance * math.cos(robot_yaw + bearing)
        target_y = robot_y + distance * math.sin(robot_yaw + bearing)

        return target_x, target_y

    def direct_chase_command(self, area, normalized_error):
        # Drive toward the visible runner when the straight path is safe
        cmd = Twist()

        if abs(normalized_error) > self.target_center_deadband:
            cmd.angular.z = self.clamp(-0.7 * normalized_error, -0.45, 0.45)
        else:
            cmd.angular.z = 0.0

        if area < 3000:
            if self.line_to_goal_blocked(self.last_target_world):
                self.set_state("PLANNING: static map blocks direct visual chase")
                return None

            drive_error_limit = max(
                self.target_drive_deadband,
                self.target_drive_max_error,
            )

            if abs(normalized_error) < drive_error_limit:
                heading_scale = self.clamp(
                    1.0 - abs(normalized_error) / drive_error_limit,
                    0.20,
                    1.0,
                )
                distance_scale = self.clamp((3000.0 - area) / 2500.0, 0.35, 1.0)
                cmd.linear.x = self.direct_chase_speed * heading_scale * distance_scale
            else:
                cmd.linear.x = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        if self.unsafe_to_drive_forward() and cmd.linear.x > 0.02:
            self.set_state("SAFETY: forward path blocked, stopping direct chase")
            return None

        return cmd

    def planned_chase_command(self, goal=None):
        # Use A* when the direct route is not enough
        if goal is None:
            goal = self.last_target_world

        if self.pose is None or goal is None:
            return None

        if self.should_replan(goal) and not self.plan_path_to_target(goal):
            return None

        return self.follow_path_command()

    def reset_lost_target_search(self):
        # Clear search state after seeing the runner again
        self.target_lost_time = None
        self.reset_scan()
        self.finished_last_seen_scan = False
        self.current_search_goal = None

    def scan_command(self):
        now = time.monotonic()

        # One full turn is enough; after that, switch to exploration
        if self.scan_start_time is None:
            self.scan_start_time = now

        scan_elapsed = now - self.scan_start_time

        if scan_elapsed >= self.scan_duration:
            self.reset_scan()
            self.set_state("SEARCHING: scan timeout reached, switching to patrol")
            return None, True

        if self.pose is None:
            self.set_state("SEARCHING: no odometry, doing timed scan")
            return self.make_turn_command(), False

        current_yaw = self.pose[2]

        if self.scan_last_yaw is None:
            self.scan_last_yaw = current_yaw
        else:
            yaw_step = abs(self.normalize_angle(current_yaw - self.scan_last_yaw))
            self.scan_turn_total += yaw_step
            self.scan_last_yaw = current_yaw

        if self.scan_turn_total >= 2.0 * math.pi - 0.25:
            self.reset_scan()
            self.set_state("SEARCHING: finished one 360 scan, switching to patrol")
            return None, True

        self.set_state("SEARCHING: scanning once around last known area")

        return self.make_turn_command(), False

    def random_free_goal(self):
        # Pick a random open cell away from borders and the current pose
        if self.pose is None:
            return None

        margin = max(self.map_border_keepout, self.robot_radius)
        margin_cells = int(math.ceil(margin / self.map_resolution))
        min_grid_x = margin_cells
        max_grid_x = self.grid_width - margin_cells - 1
        min_grid_y = margin_cells
        max_grid_y = self.grid_height - margin_cells - 1

        if min_grid_x > max_grid_x or min_grid_y > max_grid_y:
            return None

        for _ in range(max(1, self.random_goal_attempts)):
            goal_cell = (
                random.randint(min_grid_x, max_grid_x),
                random.randint(min_grid_y, max_grid_y),
            )

            if self.occupancy_grid[goal_cell[1], goal_cell[0]]:
                continue

            goal = self.grid_to_world(goal_cell[0], goal_cell[1])
            distance = math.hypot(goal[0] - self.pose[0], goal[1] - self.pose[1])

            if distance < self.random_goal_min_distance:
                continue

            return goal

        return None

    def choose_next_search_goal(self):
        # Choose the next patrol target
        goal = self.random_free_goal()

        if goal is not None:
            self.random_goal_count += 1

        return goal

    def patrol_search_command(self):
        # Explore with random reachable goals when the runner is lost
        if self.pose is None:
            return None

        if self.current_search_goal is not None:
            distance_to_goal = math.hypot(
                self.current_search_goal[0] - self.pose[0],
                self.current_search_goal[1] - self.pose[1],
            )

            if distance_to_goal < 0.60:
                self.current_search_goal = None
                self.clear_path()

        for _ in range(6):
            if self.current_search_goal is None:
                self.current_search_goal = self.choose_next_search_goal()
                self.clear_path()

            if self.current_search_goal is None:
                return None

            cmd = self.planned_chase_command(self.current_search_goal)

            if cmd is not None:
                self.set_state(
                    "SEARCHING: exploring random grid goal "
                    f"{self.random_goal_count}"
                )
                return cmd

            self.current_search_goal = None
            self.clear_path()

        return None

    def search_or_go_to_last_seen_command(self):
        now = time.monotonic()

        # Try last seen direction, then scan, then patrol with A*
        if self.target_lost_time is None:
            self.target_lost_time = now
            self.reset_scan()
            self.finished_last_seen_scan = False

        lost_for = now - self.target_lost_time

        if self.last_seen_time is not None and lost_for < self.lost_turn_duration:
            self.set_state("SEARCHING: looking toward last seen direction")
            return self.make_turn_command()

        if not self.finished_last_seen_scan:
            scan_cmd, scan_finished = self.scan_command()

            if not scan_finished:
                return scan_cmd

            self.finished_last_seen_scan = True

        cmd = self.patrol_search_command()

        if cmd is not None:
            return cmd

        self.set_state("SEARCHING: no patrol path available, stopping")

        return Twist()

    def red_mask(self, frame):
        # Keep only red pixels from the camera image
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = [
            cv2.inRange(hsv, lower, upper)
            for lower, upper in RED_HSV_RANGES
        ]
        mask = masks[0] + masks[1]
        mask = cv2.erode(mask, None, iterations=2)
        return cv2.dilate(mask, None, iterations=2)

    def draw_status_text(self, frame, text, color, scale):
        # Overlay the current behavior on the debug camera image
        cv2.putText(
            frame,
            text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            2,
        )

    def publish_debug_frame(self, frame):
        # Send the annotated camera image
        self.debug_image_pub.publish(
            self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        )

    def publish_search_debug(self, frame, text, scale=1.0):
        # Show search state on the camera debug view
        self.draw_status_text(frame, text, (0, 255, 255), scale)
        self.publish_debug_frame(frame)

    def draw_target_debug(self, frame, contour, target_x, target_y, center_x, area, error):
        # Draw the detected runner and center error.
        height = frame.shape[0]
        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
        cv2.circle(frame, (target_x, target_y), 8, (255, 0, 0), -1)
        cv2.line(
            frame,
            (int(center_x), 0),
            (int(center_x), height),
            (255, 255, 255),
            1,
        )
        self.draw_status_text(
            frame,
            f"TRACKING area={area:.0f} error={error:.2f}",
            (0, 255, 0),
            0.7,
        )

    def image_callback(self, msg):
        # Main loop: detect the runner, choose a behavior, publish velocity
        self.update_dead_reckoning()

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        debug_frame = frame.copy()

        _, width, _ = frame.shape
        center_x = width / 2

        mask = self.red_mask(frame)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding="mono8"))

        if len(contours) == 0:
            cmd = self.search_or_go_to_last_seen_command()
            self.publish_command(cmd)
            self.publish_search_debug(debug_frame, "SEARCHING")
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 80:
            self.set_state(f"SEARCHING: red noise detected, area={area:.1f}")

            cmd = self.search_or_go_to_last_seen_command()
            self.publish_command(cmd)
            self.publish_search_debug(debug_frame, "SEARCHING - RED TOO SMALL", 0.8)
            return

        moments = cv2.moments(largest)

        if moments["m00"] == 0:
            return

        target_x = int(moments["m10"] / moments["m00"])
        target_y = int(moments["m01"] / moments["m00"])

        error_x = target_x - center_x
        normalized_error = error_x / center_x
        if abs(normalized_error) > 0.05:
            self.last_search_turn = -1.0 if normalized_error > 0.0 else 1.0

        target_world = self.estimate_target_world_position(area, normalized_error)

        if target_world is not None:
            self.last_target_world = target_world
            self.last_seen_time = time.monotonic()
            self.reset_lost_target_search()

        self.draw_target_debug(
            debug_frame,
            largest,
            target_x,
            target_y,
            center_x,
            area,
            normalized_error,
        )

        if area < 3000:
            direct_cmd = self.direct_chase_command(area, normalized_error)

            if direct_cmd is not None:
                self.clear_path()
                self.current_search_goal = None
                self.set_state(
                    f"TRACKING: direct visual chase, "
                    f"area={area:.0f}, error={normalized_error:.2f}"
                )
                cmd = direct_cmd
            else:
                planned_cmd = self.planned_chase_command()

                if planned_cmd is not None:
                    self.set_state(
                        f"PLANNING: obstacle in direct path, waypoints={len(self.path)}, "
                        f"area={area:.0f}, error={normalized_error:.2f}"
                    )
                    cmd = planned_cmd
                else:
                    self.set_state("SAFETY: direct path blocked and no A* path")
                    cmd = Twist()
        else:
            self.set_state("CAUGHT: runner is very close")
            cmd = Twist()

        self.publish_command(cmd)
        self.publish_debug_frame(debug_frame)
