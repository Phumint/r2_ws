#!/usr/bin/env python3
"""Closed-loop odometry accuracy score for the R2 mecanum base.

Drives a closed square using translation only (no rotation, which a mecanum base
can do and a differential base cannot) and reports how far the estimate has
drifted from Gazebo ground truth by the time the robot returns to its start.

Two numbers matter:

  loop closure error - how far ground truth says the robot is from where it
                       started. This measures the DRIVE, not the estimator: a
                       large value means the contact model is slipping.
  estimate error     - how far the EKF estimate is from ground truth. This is
                       the odometry quality figure, and the one to watch when
                       tuning covariances in r2_bringup/config/ekf.yaml.

Usage:
  ros2 run r2_gazebo odometry_check.py
  ros2 run r2_gazebo odometry_check.py --side 2.0 --speed 0.3
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdometryCheck(Node):

    def __init__(self, args):
        # See the note in holonomic_check.py: without sim time the controller
        # discards every reference as stale.
        super().__init__(
            'odometry_check',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        self._pub = self.create_publisher(TwistStamped, args.reference_topic, 10)
        self._msgs = {}

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, args.truth_topic,
                                 lambda m: self._msgs.__setitem__('truth', m), sensor_qos)
        self.create_subscription(Odometry, args.estimate_topic,
                                 lambda m: self._msgs.__setitem__('est', m), 10)

    def _pose(self, key):
        m = self._msgs.get(key)
        if m is None:
            return None
        p = m.pose.pose
        return (p.position.x, p.position.y, yaw_of(p.orientation))

    def wait(self, timeout=25.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and len(self._msgs) < 2:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() > deadline:
                return False
        return True

    def _drive(self, vx, vy, seconds):
        """Command a body velocity for `seconds` of SIMULATED time."""
        start = self.get_clock().now()
        while rclpy.ok():
            if (self.get_clock().now() - start).nanoseconds >= seconds * 1e9:
                return
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist = Twist()
            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(0.015)

    def _settle(self, seconds=2.0):
        start = self.get_clock().now()
        while rclpy.ok():
            if (self.get_clock().now() - start).nanoseconds >= seconds * 1e9:
                return
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(0.015)

    def run(self):
        side, v = self.args.side, self.args.speed
        leg = side / v

        t0, e0 = self._pose('truth'), self._pose('est')
        print(f'\nsquare {side} m per side at {v} m/s, translation only '
              f'({leg:.1f}s per leg)\n')
        print(f'{"leg":10s} {"truth (x, y, yaw)":34s} {"estimate (x, y, yaw)":34s}')

        def report(tag):
            t, e = self._pose('truth'), self._pose('est')
            print(f'{tag:10s} '
                  f'({t[0]:+.3f}, {t[1]:+.3f}, {math.degrees(t[2]):+7.2f}deg)      '
                  f'({e[0]:+.3f}, {e[1]:+.3f}, {math.degrees(e[2]):+7.2f}deg)')
            return t, e

        report('start')
        # Forward, left, back, right. No rotation anywhere: the heading should
        # stay put for the whole square, which is the mecanum-specific part.
        for tag, (vx, vy) in [('+x', (v, 0.0)), ('+y', (0.0, v)),
                              ('-x', (-v, 0.0)), ('-y', (0.0, -v))]:
            self._drive(vx, vy, leg)
            self._settle()
            report(tag)

        t1, e1 = self._pose('truth'), self._pose('est')

        closure = math.hypot(t1[0] - t0[0], t1[1] - t0[1])
        # Estimate error relative to ground truth, both measured as displacement
        # from their own starting pose, so a non-zero initial offset cancels.
        dtx, dty = t1[0] - t0[0], t1[1] - t0[1]
        dex, dey = e1[0] - e0[0], e1[1] - e0[1]
        est_err = math.hypot(dex - dtx, dey - dty)
        yaw_err = math.degrees((e1[2] - e0[2]) - (t1[2] - t0[2]))
        path = 4 * side

        print(f'\npath length            {path:.2f} m')
        print(f'loop closure (truth)   {closure:.3f} m   '
              f'({100 * closure / path:.2f}% of path)   <- drive/contact quality')
        print(f'estimate error (EKF)   {est_err:.3f} m   '
              f'({100 * est_err / path:.2f}% of path)   <- odometry quality')
        print(f'estimate yaw error     {yaw_err:+.2f} deg\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--side', type=float, default=2.0)
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--reference-topic', default='/mecanum_drive_controller/reference')
    ap.add_argument('--truth-topic', default='/ground_truth/odom')
    ap.add_argument('--estimate-topic', default='/odometry/filtered')
    args = ap.parse_args()

    rclpy.init()
    node = OdometryCheck(args)
    if not node.wait():
        print(f'ERROR: need both {args.truth_topic} and {args.estimate_topic}. '
              f'Is sim.launch.xml running with localization:=true and '
              f'use_ground_truth:=true?', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.run()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
