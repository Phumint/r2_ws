#!/usr/bin/env python3
"""Measure how the R2 base stops when commanded to zero velocity.

Background: the wheel joints reach exactly 0.000 rad/s within ~0.2 s of a zero
command, but the chassis does not always stop with them. Braking is limited by
the wheel/ground contact, not by wheel torque, so the coast distance depends on
roller contact state rather than on anything the controller does.

Three command profiles, because they are genuinely different regimes:

  --profile continuous   publish at 50 Hz, then a continuous stream of zeros.
                         What a nav stack or a well-behaved teleop would do.
  --profile ramp         as above, but ramp the command down over --ramp-time
                         instead of stepping to zero.
  --profile burst        emulate teleop_twist_keyboard: ONE message per
                         "keypress", relying on keyboard auto-repeat, with the
                         controller's reference_timeout doing the rest. This is
                         what a human at the keyboard actually produces and it
                         is NOT the same as `continuous`.

Reports coast distance and, more importantly, how often the robot fails to stop
at all (still moving after the settle window).

Usage:
  ros2 run r2_gazebo stop_check.py --trials 6
  ros2 run r2_gazebo stop_check.py --profile burst --trials 6
  ros2 run r2_gazebo stop_check.py --profile ramp --ramp-time 0.3
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

STOPPED_SPEED = 0.01  # m/s below which we call it stopped


class StopCheck(Node):

    def __init__(self, args):
        # Without sim time the controller discards every reference as stale:
        # it enforces reference_timeout against ITS clock, which is sim time.
        super().__init__(
            'stop_check',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        self._pub = self.create_publisher(TwistStamped, args.reference_topic, 10)
        self._truth = None
        self.create_subscription(
            Odometry, args.truth_topic, self._on_truth,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _on_truth(self, msg):
        self._truth = msg

    def wait_for_truth(self, timeout=25.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self._truth is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() > deadline:
                return False
        return True

    def speed(self):
        t = self._truth.twist.twist.linear
        return math.hypot(t.x, t.y)

    def position(self):
        p = self._truth.pose.pose.position
        return (p.x, p.y)

    def _send(self, vx):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        self._pub.publish(msg)

    def _spin_until(self, seconds, on_tick=None):
        """Run for `seconds` of SIMULATED time.

        Bounded by the sim clock, not an iteration count: rclpy.spin_once
        returns immediately whenever a callback is pending, so a fixed step
        count burns through in a fraction of the intended window.
        """
        start = self.get_clock().now()
        while rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= seconds:
                return
            if on_tick is not None:
                on_tick(elapsed)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(0.015)

    def _cruise_continuous(self, v, seconds):
        self._spin_until(seconds, lambda _: self._send(v))

    def _cruise_burst(self, v, seconds):
        """One message per simulated keypress, like teleop_twist_keyboard.

        The node publishes exactly once per key event and has no repeat timer,
        so a human holding a key produces the terminal's auto-repeat pattern:
        one press, a long initial delay, then a steady repeat.
        """
        state = {'next': 0.0, 'first': True}

        def tick(elapsed):
            if elapsed >= state['next']:
                self._send(v)
                gap = self.args.autorepeat_delay if state['first'] \
                    else 1.0 / self.args.autorepeat_rate
                state['first'] = False
                state['next'] = elapsed + gap

        self._spin_until(seconds, tick)

    def run_trial(self, v, cruise_s, settle_s):
        # Come to rest first so every trial starts the same way.
        self._spin_until(1.0, lambda _: self._send(0.0))

        if self.args.profile == 'burst':
            self._cruise_burst(v, cruise_s)
        else:
            self._cruise_continuous(v, cruise_s)

        v_release = self.speed()
        p0 = self.position()

        if self.args.profile == 'ramp':
            rt = self.args.ramp_time
            self._spin_until(rt, lambda e: self._send(v * max(0.0, 1.0 - e / rt)))
        elif self.args.profile == 'burst':
            # A single zero message, exactly as pressing 'k' produces.
            self._send(0.0)

        # Settle. Under 'burst' we deliberately send nothing more: the
        # controller's reference_timeout is what holds the wheels at zero.
        if self.args.profile == 'burst':
            self._spin_until(settle_s)
        else:
            self._spin_until(settle_s, lambda _: self._send(0.0))

        return (v_release,
                math.dist(self.position(), p0),
                self.speed())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=6)
    ap.add_argument('--speed', type=float, default=0.4)
    ap.add_argument('--cruise-time', type=float, default=4.0)
    ap.add_argument('--settle-time', type=float, default=2.5)
    ap.add_argument('--profile', choices=['continuous', 'ramp', 'burst'],
                    default='continuous')
    ap.add_argument('--ramp-time', type=float, default=0.3)
    ap.add_argument('--autorepeat-delay', type=float, default=0.5,
                    help='burst: delay before the terminal starts repeating [s]')
    ap.add_argument('--autorepeat-rate', type=float, default=30.0,
                    help='burst: key repeat rate once repeating [Hz]')
    ap.add_argument('--reference-topic',
                    default='/mecanum_drive_controller/reference')
    ap.add_argument('--truth-topic', default='/ground_truth/odom')
    args = ap.parse_args()

    rclpy.init()
    node = StopCheck(args)
    if not node.wait_for_truth():
        print(f'ERROR: nothing on {args.truth_topic}. Is the simulation running '
              f'with use_ground_truth:=true?', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    label = args.profile
    if args.profile == 'ramp':
        label += f' ({args.ramp_time}s)'
    print(f'\nprofile: {label}   cruise {args.speed} m/s for {args.cruise_time}s   '
          f'settle {args.settle_time}s\n')
    print(f'{"trial":>5s} {"v_release":>10s} {"coast_m":>9s} {"v_end":>8s}  result')

    dists, failures = [], 0
    for i in range(args.trials):
        v0, dist, v_end = node.run_trial(
            args.speed, args.cruise_time, args.settle_time)
        dists.append(dist)
        stopped = v_end < STOPPED_SPEED
        failures += 0 if stopped else 1
        print(f'{i + 1:5d} {v0:10.3f} {dist:9.4f} {v_end:8.4f}  '
              f'{"stopped" if stopped else "STILL MOVING"}')

    mean = sum(dists) / len(dists)
    print(f'\ncoast distance  min {min(dists):.3f}  max {max(dists):.3f}  '
          f'mean {mean:.3f} m')
    print(f'failed to stop  {failures}/{args.trials}')
    # v^2 / 2d, a useful single number to compare configurations with.
    if mean > 0:
        print(f'implied mean deceleration  {args.speed ** 2 / (2 * mean):.2f} m/s^2\n')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
