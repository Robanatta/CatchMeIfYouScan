import rclpy

from robomaster_chase.controller import ControllerNode
from geometry_msgs.msg import Twist

import select
import sys
import termios
import time
import tty


HELP = """\
keyboard_controller: hold arrow keys to drive (gamepad-style).
  Up / Down   : forward / backward
  Left / Right: turn left / right (combine with Up/Down for arcs)
  space       : emergency stop
  q / Ctrl-C  : quit

Terminals don't emit key-release events, so we approximate hold-to-drive:
each keypress refreshes a per-axis timer. While the timer is fresh the axis
runs at full speed; once auto-repeat stops, the axis decays back to zero.
"""


class KeyboardController(ControllerNode):
    # 20 Hz republish keeps the bridge's watchdog happy and matches goto_controller.
    UPDATE_STEP = 1 / 20

    # Velocity caps. Same as goto_controller's MAX_* values.
    MAX_LINEAR_SPEED = 0.30   # [m/s]
    MAX_ANGULAR_SPEED = 0.60  # [rad/s]

    # How long an axis stays "held" after the most recent keypress on it.
    # macOS key auto-repeat is ~30 Hz once warmed up (~0.5s initial delay,
    # then ~33ms between repeats), so 0.15s is comfortably longer than the
    # gap between repeats but short enough that release feels instant.
    HOLD_TIMEOUT = 0.15  # [s]

    def __init__(self):
        super().__init__("keyboard_controller", update_step=self.UPDATE_STEP)

        # Per-direction "last pressed" timestamps. An axis is active iff one of
        # its two directions was pressed within HOLD_TIMEOUT seconds.
        self._last_press = {"fwd": 0.0, "back": 0.0, "left": 0.0, "right": 0.0}
        self.should_quit = False

        # Save terminal state so we can restore line-buffered mode on exit.
        self._stdin_fd = sys.stdin.fileno()
        self._old_termios = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

        self.get_logger().info(HELP)

    def restore_terminal(self):
        termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_termios)

    def update_callback(self):
        self._poll_keys()

        if self.should_quit:
            self.stop()
            rclpy.shutdown()
            return

        now = time.monotonic()
        cmd = Twist()
        cmd.linear.x = self._axis_velocity(now, "fwd", "back", self.MAX_LINEAR_SPEED)
        cmd.angular.z = self._axis_velocity(now, "left", "right", self.MAX_ANGULAR_SPEED)
        self.vel_publisher.publish(cmd)

    def _axis_velocity(self, now, pos_key, neg_key, max_speed):
        # An axis fires at +max if its positive direction is the most recent
        # still-fresh press; -max if its negative direction is; 0 otherwise.
        pos_fresh = (now - self._last_press[pos_key]) < self.HOLD_TIMEOUT
        neg_fresh = (now - self._last_press[neg_key]) < self.HOLD_TIMEOUT
        if pos_fresh and not neg_fresh:
            return max_speed
        if neg_fresh and not pos_fresh:
            return -max_speed
        if pos_fresh and neg_fresh:
            # Both held simultaneously: whichever was pressed more recently wins.
            return max_speed if self._last_press[pos_key] >= self._last_press[neg_key] else -max_speed
        return 0.0

    def _poll_keys(self):
        # Drain whatever is buffered without blocking the timer.
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if not ready:
                return
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                # Escape sequence: arrows arrive as ESC '[' <A|B|C|D>.
                seq = sys.stdin.read(2)
                self._handle_arrow(seq)
            else:
                self._handle_char(ch)

    def _handle_arrow(self, seq):
        now = time.monotonic()
        if seq == "[A":     # Up
            self._last_press["fwd"] = now
        elif seq == "[B":   # Down
            self._last_press["back"] = now
        elif seq == "[D":   # Left
            self._last_press["left"] = now
        elif seq == "[C":   # Right
            self._last_press["right"] = now

    def _handle_char(self, ch):
        if ch == " ":
            # Emergency stop: invalidate every axis immediately.
            for k in self._last_press:
                self._last_press[k] = 0.0
        elif ch in ("q", "\x03"):  # q or Ctrl-C
            self.should_quit = True


def main():
    rclpy.init(args=sys.argv)
    node = KeyboardController()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.restore_terminal()


if __name__ == "__main__":
    main()
