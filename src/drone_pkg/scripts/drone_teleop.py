#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# drone_teleop.py
# Phase 5 — Altitude Hold PID teleop
# Q/E nudge target height. WASD = lateral. SPACE = lock height. X = kill.
# ─────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios, select, threading

HELP = """
╔══════════════════════════════════════════╗
║      DRONE TELEOP — ALTITUDE HOLD        ║
╠══════════════════════════════════════════╣
║  Q        →  target height UP   (+0.2m)  ║
║  E        →  target height DOWN (-0.2m)  ║
║  SPACE    →  lock current height         ║
║  X        →  KILL motors                 ║
║                                          ║
║  W / S    →  pitch forward / backward    ║
║  A / D    →  roll  left    / right       ║
║  Z / C    →  yaw   left    / right       ║
║                                          ║
║  Ctrl+C   →  quit                        ║
╚══════════════════════════════════════════╝
"""

LATERAL_STEP  = 0.25
LATERAL_DECAY = 0.80
TELEOP_HZ     = 20


class TeleopNode(Node):
    def __init__(self):
        super().__init__("drone_teleop")
        self.pub   = self.create_publisher(Twist, "/drone/cmd", 10)
        self.timer = self.create_timer(1.0 / TELEOP_HZ, self._publish)

        self.target_z  = 0.0   # metres — sent as linear.z steps
        self.fx        = 0.0
        self.fy        = 0.0
        self.tz        = 0.0
        self._pending_z = 0.0  # one-shot nudge value

        print(HELP)
        self._print_status()

    def _publish(self):
        self.fx *= LATERAL_DECAY
        self.fy *= LATERAL_DECAY
        self.tz *= LATERAL_DECAY

        if abs(self.fx) < 0.01: self.fx = 0.0
        if abs(self.fy) < 0.01: self.fy = 0.0
        if abs(self.tz) < 0.01: self.tz = 0.0

        msg = Twist()
        msg.linear.z  = self._pending_z   # nudge or special code
        msg.linear.x  = self.fx
        msg.linear.y  = self.fy
        msg.angular.z = self.tz
        self.pub.publish(msg)

        self._pending_z = 0.0  # consumed — don't repeat nudge

    def handle_key(self, key: str):
        k = key.lower()

        if   k == 'q':  self.target_z += 0.2; self._pending_z =  1.0
        elif k == 'e':  self.target_z -= 0.2; self._pending_z = -1.0; self.target_z = max(0.0, self.target_z)
        elif k == ' ':  self._pending_z = 99.0   # special: lock current height
        elif k == 'x':  self._pending_z = -99.0; self.target_z = 0.0  # kill

        elif k == 'w':  self.fx = min( 1.0, self.fx + LATERAL_STEP)
        elif k == 's':  self.fx = max(-1.0, self.fx - LATERAL_STEP)
        elif k == 'd':  self.fy = min( 1.0, self.fy + LATERAL_STEP)
        elif k == 'a':  self.fy = max(-1.0, self.fy - LATERAL_STEP)
        elif k == 'z':  self.tz = min( 1.0, self.tz + LATERAL_STEP)
        elif k == 'c':  self.tz = max(-1.0, self.tz - LATERAL_STEP)

        self._print_status()

    def _print_status(self):
        print(
            f"\r  target_z={self.target_z:.1f}m  |"
            f"  fx={self.fx:+.2f}  fy={self.fy:+.2f}  tz={self.tz:+.2f}   ",
            end='', flush=True
        )


def get_key(saved_settings):
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_settings)
    return key


def main():
    rclpy.init()
    node = TeleopNode()

    saved_settings = termios.tcgetattr(sys.stdin)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            key = get_key(saved_settings)
            if key == '\x03':
                break
            if key:
                node.handle_key(key)
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_settings)
        msg = Twist()
        node.pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()