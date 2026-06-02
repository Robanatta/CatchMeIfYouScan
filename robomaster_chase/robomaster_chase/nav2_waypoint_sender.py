"""Send a list of MAP-frame waypoints to Nav2 so Robot A tours them autonomously.

Nav2 is goal-driven -- it doesn't move until given a goal. This node hands the
whole waypoint list to Nav2's waypoint_follower (via nav2_simple_commander's
followWaypoints), and Nav2 plans + drives to each point in order, routing around
walls and dodging obstacles between legs. Optionally loops forever.

This is the autonomous point-to-point driver for the Nav2 stack (replacing the
hand-rolled goto/avoiding controllers). Run it alongside nav2.launch.py, or let
nav2.launch.py start it (auto_send arg).

Params:
  waypoints   flat [x0,y0, x1,y1, ...] in the MAP frame (default one point at 1,0)
  loop        repeat the tour forever (default False -> visit once and stop)
  frame_id    goal frame (default "map")

Notes:
  - We localize with slam_toolbox, NOT amcl, so waitUntilNav2Active is called
    with localizer="slam_toolbox" (the default 'amcl' would block forever).
  - Each waypoint's orientation is set to face the NEXT waypoint (yaw along the
    leg); the last keeps the previous heading. Nav2's goal_checker yaw tolerance
    is loose, so this is just a sensible default.
"""

import sys
from math import atan2, sin, cos

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def _yaw_to_quat(yaw):
    return (0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0))  # (x, y, z, w)


class Nav2WaypointSender(Node):
    def __init__(self):
        super().__init__("nav2_waypoint_sender")
        self.declare_parameter("waypoints", [1.0, 0.0])
        self.declare_parameter("loop", False)
        self.declare_parameter("frame_id", "map")

        flat = list(self.get_parameter("waypoints").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.frame_id = self.get_parameter("frame_id").value

        if len(flat) == 0 or len(flat) % 2 != 0:
            self.get_logger().error(
                f"waypoints must be a non-empty flat list of even length "
                f"[x0,y0,x1,y1,...], got {flat!r}. Falling back to default."
            )
            flat = [1.0, 0.0]
        self.points = [(float(flat[i]), float(flat[i + 1]))
                       for i in range(0, len(flat), 2)]

        self.nav = BasicNavigator()

    def _make_poses(self):
        """Build PoseStamped goals, each facing the next waypoint."""
        poses = []
        n = len(self.points)
        for i, (x, y) in enumerate(self.points):
            nx, ny = self.points[i + 1] if i + 1 < n else self.points[i]
            yaw = atan2(ny - y, nx - x) if (nx, ny) != (x, y) else 0.0
            qx, qy, qz, qw = _yaw_to_quat(yaw)

            p = PoseStamped()
            p.header.frame_id = self.frame_id
            p.header.stamp = self.nav.get_clock().now().to_msg()
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.x = qx
            p.pose.orientation.y = qy
            p.pose.orientation.z = qz
            p.pose.orientation.w = qw
            poses.append(p)
        return poses

    def run(self):
        # Wait for the navigation stack only (bt_navigator is a managed
        # lifecycle node with a get_state service). We do NOT wait on the
        # localizer: slam_toolbox in async mode is NOT a lifecycle node, so it
        # has no get_state service and waitUntilNav2Active(localizer="...")
        # would block forever. The map->odom transform from slam_toolbox is in
        # place by the time bt_navigator is active.
        self.get_logger().info(
            f"waiting for Nav2 (bt_navigator) to become active "
            f"({len(self.points)} waypoint(s) to tour, loop={self.loop})..."
        )
        self.nav._waitForNodeToActivate("bt_navigator")
        self.get_logger().info("Nav2 active -> sending waypoints.")

        while rclpy.ok():
            poses = self._make_poses()
            self.nav.followWaypoints(poses)

            while not self.nav.isTaskComplete():
                fb = self.nav.getFeedback()
                if fb is not None:
                    self.get_logger().info(
                        f"heading to waypoint "
                        f"{fb.current_waypoint + 1}/{len(poses)}",
                        throttle_duration_sec=2.0,
                    )
                rclpy.spin_once(self.nav, timeout_sec=0.1)

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("waypoint tour complete.")
            elif result == TaskResult.CANCELED:
                self.get_logger().warning("waypoint tour canceled.")
                break
            else:
                self.get_logger().warning(
                    "waypoint tour failed (some waypoints unreachable)."
                )

            if not self.loop:
                break
            self.get_logger().info("looping the tour again.")


def main():
    rclpy.init(args=sys.argv)
    node = Nav2WaypointSender()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
