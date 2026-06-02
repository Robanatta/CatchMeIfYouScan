import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import sys


class ControllerNode(Node):
    def __init__(self, *args, update_step=1 / 60, **kwargs):
        super().__init__(*args, **kwargs)

        # Period of the update timer
        self.update_step = update_step  # [s]

        # Latest odometry pose and velocity (None until the first message)
        self.odom_pose = None
        self.odom_velocity = None

        # Publisher for velocity commands and subscriber for odometry.
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom_subscriber = self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        # Optional map-frame pose, looked up via tf2. Controllers that drive in
        # the map frame (localized against a saved SLAM map) set use_map_frame
        # true and read map_pose_2d() instead of the drifting odom_pose. Off by
        # default so odom-frame controllers are unaffected.
        self.declare_parameter("use_map_frame", False)
        self.declare_parameter("tf_prefix", "rm0")
        self.declare_parameter("map_frame", "map")
        self.use_map_frame = self.get_parameter("use_map_frame").value
        prefix = self.get_parameter("tf_prefix").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = f"{prefix}/base_link" if prefix else "base_link"

        # Only spin up the TF listener when we actually need map-frame poses.
        if self.use_map_frame:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.get_logger().info(
                f"controller: map-frame mode, looking up "
                f"{self.map_frame} -> {self.base_frame} via tf2"
            )

    def start(self):
        self.timer = self.create_timer(self.update_step, self.update_callback)

    def stop(self):
        self.vel_publisher.publish(Twist())

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist

        pose2d = self.pose3d_to_2d(self.odom_pose)
        self.get_logger().debug(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
            throttle_duration_sec=0.5,
        )

    def pose3d_to_2d(self, pose3):
        quaternion = (pose3.orientation.w, pose3.orientation.x, pose3.orientation.y, pose3.orientation.z)
        _, _, yaw = euler_from_quaternion(quaternion)
        return (pose3.position.x, pose3.position.y, yaw)

    def map_pose_2d(self):
        """(x, y, yaw) of base_link in the map frame, or None if TF isn't ready.

        Requires use_map_frame=true (sets up the TF listener) and a localizer
        publishing map -> <prefix>/odom (slam_toolbox localization mode or the
        ground-truth localizer). The latest available transform is requested
        (Time()) rather than a stamped one, matching ground_truth_localizer:
        the robot moves slowly relative to the TF rate, so latest-vs-exact is
        negligible and it avoids extrapolation errors from mixed clocks.
        """
        if not self.use_map_frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warning(
                f"no {self.map_frame}->{self.base_frame} transform yet: {e}",
                throttle_duration_sec=2.0,
            )
            return None
        q = tf.transform.rotation
        _, _, yaw = euler_from_quaternion((q.w, q.x, q.y, q.z))
        return (tf.transform.translation.x, tf.transform.translation.y, yaw)

    def update_callback(self):
        # Default: hold position. Subclasses override this.
        self.vel_publisher.publish(Twist())


def main():
    rclpy.init(args=sys.argv)
    node = ControllerNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()


if __name__ == "__main__":
    main()
