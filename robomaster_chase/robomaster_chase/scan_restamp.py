"""Restamp the Coppelia LaserScan onto the ROS wall clock.

The Coppelia lidar script stamps /rm0/scan with sim time (starts at 0), while
the robomaster_ros bridge stamps its TF with ROS wall-clock time. slam_toolbox
needs the scan and the TF on the same clock to look up the robot pose at each
scan's time.

Forcing the whole system onto sim time (use_sim_time + /clock) breaks the
RoboMaster SDK's connection keepalive (it connect/disconnect loops). So instead
we keep everything on wall-clock and fix only the scan: subscribe to the
sim-stamped /rm0/scan, overwrite its header.stamp with now() (wall clock), and
republish as /rm0/scan_stamped. SLAM consumes the restamped topic.

This adds at most one scan period of latency, which is negligible for mapping.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanRestamp(Node):
    def __init__(self):
        super().__init__("scan_restamp")
        self.declare_parameter("input_topic", "scan")
        self.declare_parameter("output_topic", "scan_stamped")
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        # Coppelia's simROS2 publishes RELIABLE; a BEST_EFFORT sub is
        # compatible and robust. Republish RELIABLE so slam_toolbox (default
        # RELIABLE) connects without QoS gymnastics.
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(LaserScan, out_topic, 10)
        self.create_subscription(LaserScan, in_topic, self._cb, sub_qos)

        self.get_logger().info(
            f"scan_restamp: {in_topic} -> {out_topic}, stamping with now() "
            f"(wall clock)"
        )

    def _cb(self, msg: LaserScan):
        # Stamp with now() (wall clock). The TF lookup needs odom->base_link to
        # bracket this time; the bridge publishes that continuously on
        # wall-clock, and slam_toolbox's transform_timeout (0.2 s) lets it wait
        # briefly for the match instead of dropping the scan.
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    node = ScanRestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
