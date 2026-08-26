#!/usr/bin/env python3
"""Open-loop holonomic motion check for the R2 mecanum base.

Commands a constant body velocity for a fixed duration and reports what the
robot actually did, measured against Gazebo ground truth. This is the test that
decides whether the mecanum contact model is good enough, and it is the number
to re-run after any change to wheel friction, roller geometry or mass.

Interpreting the output:
  forward  - dy and dyaw should be ~0
  strafe   - dx and dyaw should be ~0; |dy| is the one that matters
  rotate   - dx and dy should be ~0
  diagonal - dx ~= dy, dyaw ~0

`eff` is measured displacement divided by commanded displacement. A strafe
efficiency well below 1.0 means the wheels are slipping rather than driving
sideways, which is the expected failure mode of an approximate contact model.

Usage:
  ros2 run r2_gazebo holonomic_check.py
  ros2 run r2_gazebo holonomic_check.py --duration 4.0 --speed 0.4
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_of(q):
    """Extract yaw from a quaternion (planar robot, so roll/pitch are ignored)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class HolonomicCheck(Node):

    def __init__(self, reference_topic, truth_topic):
        # use_sim_time is not optional here. The controller enforces
        # reference_timeout by comparing header.stamp against ITS clock, which
        # is sim time. A node stamping with wall-clock time produces references
        # that look hours stale, so every one is discarded and the robot sits
        # still while looking perfectly healthy.
        super().__init__(
            'holonomic_check',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self._pub = self.create_publisher(TwistStamped, reference_topic, 10)

        # Ground truth arrives over the bridge with SENSOR_DATA QoS.
        self._truth = None
        self.create_subscription(
            Odometry, truth_topic, self._on_truth,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _on_truth(self, msg):
        self._truth = msg

    def pose(self):
        if self._truth is None:
            return None
        p = self._truth.pose.pose
        return (p.position.x, p.position.y, yaw_of(p.orientation))

    def wait_for_truth(self, timeout=20.0):
        # Wall-clock deadline on purpose: sim time is not yet trustworthy here,
        # since /clock may not have arrived.
        import time
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self._truth is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() > deadline:
                return False
        return True

    def _spin_for(self, seconds, twist=None):
        """Publish `twist` (or nothing) at ~50 Hz for `seconds` of SIMULATED time.

        Bounded by the sim clock, not by an iteration count: rclpy.spin_once
        returns immediately whenever a callback is pending, so a fixed step
        count burns through in a fraction of the intended window and every
        measurement comes out near zero. Sim time also keeps the result
        independent of real-time factor.
        """
        import time
        start = self.get_clock().now()
        while rclpy.ok():
            if (self.get_clock().now() - start).nanoseconds >= seconds * 1e9:
                return
            if twist is not None:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist = twist
                self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(0.015)

    def run_case(self, name, vx, vy, wz, duration, settle=1.5):
        from geometry_msgs.msg import Twist

        # Let the previous case's momentum bleed off before measuring.
        self._spin_for(settle)

        start = self.pose()

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self._spin_for(duration, cmd)

        # Measure at the END OF THE COMMAND WINDOW, before coasting. Sampling
        # after the settle would add the coast-down distance to the result and
        # report efficiencies above 1.0 for a robot that merely has momentum.
        end = self.pose()

        # Now let it stop, so the next case starts from rest. reference_timeout
        # zeroes the wheels once we stop publishing.
        self._spin_for(settle)

        dx_w = end[0] - start[0]
        dy_w = end[1] - start[1]
        dyaw = wrap(end[2] - start[2])

        # Rotate the world-frame displacement back into the body frame the
        # command was expressed in, otherwise a robot that also rotated would
        # look like it translated in the wrong direction.
        c, s = math.cos(-start[2]), math.sin(-start[2])
        dx = c * dx_w - s * dy_w
        dy = s * dx_w + c * dy_w

        want_x, want_y, want_yaw = vx * duration, vy * duration, wz * duration

        def eff(actual, want):
            return f'{actual / want:5.2f}' if abs(want) > 1e-9 else '    -'

        print(f'{name:9s} cmd(vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f})  '
              f'dx={dx:+.3f} dy={dy:+.3f} dyaw={math.degrees(dyaw):+7.2f}deg  '
              f'| eff_x={eff(dx, want_x)} eff_y={eff(dy, want_y)} '
              f'eff_yaw={eff(dyaw, want_yaw)}')
        return dx, dy, dyaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=3.0)
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--yaw-rate', type=float, default=0.5)
    ap.add_argument('--reference-topic',
                    default='/mecanum_drive_controller/reference')
    ap.add_argument('--truth-topic', default='/ground_truth/odom')
    args = ap.parse_args()

    rclpy.init()
    node = HolonomicCheck(args.reference_topic, args.truth_topic)

    if not node.wait_for_truth():
        print(f'ERROR: no messages on {args.truth_topic}. Is the simulation '
              f'running with use_ground_truth:=true?', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    v, d, w = args.speed, args.duration, args.yaw_rate
    print(f'\ncommanded speed {v} m/s, yaw rate {w} rad/s, {d}s per case\n')

    node.run_case('forward', v, 0.0, 0.0, d)
    node.run_case('strafe', 0.0, v, 0.0, d)
    node.run_case('rotate', 0.0, 0.0, w, d)
    node.run_case('diagonal', v, v, 0.0, d)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
