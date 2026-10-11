#!/usr/bin/env python3
"""
This node publishes a velocity command (/cmd_vel) to drive the robot in a 
trajectory.

Usage:
    ros2 launch kalman_demo kalman_launch.py # auto (name hash)
    ros2 launch kalman_demo kalman_launch.py pattern:=circle
    ros2 launch kalman_demo kalman_launch.py pattern:=square
    ros2 launch kalman_demo kalman_launch.py pattern:=figure8
    ros2 launch kalman_demo kalman_launch.py pattern:=curvy
    ros2 launch kalman_demo kalman_launch.py pattern:=spiral
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

VALID_PATTERNS = ('circle', 'square', 'figure8', 'curvy', 'spiral')


class StudentTrajectoryNode(Node):
    def __init__(self):
        super().__init__('student_trajectory_node')

        # 'auto' (default) reproduces the original assignment's behavior:
        # the trajectory is picked from a hash of student_name. Set this
        # to one of VALID_PATTERNS to pick a trajectory explicitly instead.

        self.declare_parameter('pattern', 'auto')
        self.declare_parameter('student_name', 'Maryam Mosadegh')

        self.declare_parameter('figure8_linear', 0.25)
        self.declare_parameter('figure8_radius', 1.0)
        self.declare_parameter('curvy_linear', 0.3)
        self.declare_parameter('curvy_amplitude', 0.5)   # rad/s, weave sharpness
        self.declare_parameter('curvy_period', 6.0)       # s per left-right weave
        self.declare_parameter('spiral_linear', 0.25)
        self.declare_parameter('spiral_omega_start', 0.6)  # rad/s, tight at start
        self.declare_parameter('spiral_omega_min', 0.05)   # rad/s, widest point
        self.declare_parameter('spiral_duration', 60.0)    # s per spiral-out cycle

        self.figure8_linear = self.get_parameter('figure8_linear').value
        self.figure8_radius = self.get_parameter('figure8_radius').value
        self.curvy_linear = self.get_parameter('curvy_linear').value
        self.curvy_amplitude = self.get_parameter('curvy_amplitude').value
        self.curvy_period = self.get_parameter('curvy_period').value
        self.spiral_linear = self.get_parameter('spiral_linear').value
        self.spiral_omega_start = self.get_parameter('spiral_omega_start').value
        self.spiral_omega_min = self.get_parameter('spiral_omega_min').value
        self.spiral_duration = self.get_parameter('spiral_duration').value

        # publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        pattern = self.get_parameter('pattern').value
        student_name = self.get_parameter('student_name').value

        if pattern == 'auto':
            name_hash = sum(ord(c) for c in student_name) % 3
            pattern = {0: 'circle', 1: 'square', 2: 'figure8'}[name_hash]
            self.get_logger().info(f'Student: {student_name}')
            self.get_logger().info(
                f'pattern=auto -> hash {name_hash} -> "{pattern}"')
        elif pattern not in VALID_PATTERNS:
            self.get_logger().warn(
                f'Unknown pattern "{pattern}", falling back to "circle". '
                f'Valid patterns: {", ".join(VALID_PATTERNS)}')
            pattern = 'circle'
        else:
            self.get_logger().info(f'pattern explicitly set to "{pattern}"')

        self._trajectory_fn = {
            'circle': self._circle,
            'square': self._square,
            'figure8': self._figure8,
            'curvy': self._curvy,
            'spiral': self._spiral,
        }[pattern]
        self.get_logger().info(f'Trajectory: {pattern.upper()}')

        self.start_time = self.get_clock().now()
        self.create_timer(0.1, self.tick)  # 10 Hz, same as rospy.Rate(10)

    def _elapsed(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def _circle(self, t):
        return 0.3, 0.3  # v = 0.3 m/s, omega = 0.3 rad/s

    def _square(self, t):
        cycle_time = t % 5.0  # 4s forward + 1s turn
        if cycle_time < 4.0:
            return 0.2, 0.0
        return 0.0, math.pi / 2.0  # 90 deg/s

    def _figure8(self, t):
        v = self.figure8_linear
        omega = v / self.figure8_radius

        half_circle_time = math.pi / omega
        full_circle_time = 2 * math.pi / omega
        total_cycle_time = half_circle_time + 3 * full_circle_time

        cycle_time = t % total_cycle_time
        if cycle_time < half_circle_time:
            w = omega  # half left circle
        elif cycle_time < half_circle_time + full_circle_time:
            w = -omega  # full right circle
        elif cycle_time < half_circle_time + 2 * full_circle_time:
            w = omega  # full left circle
        else:
            w = -omega  # full right circle again
        return v, w

    def _curvy(self, t):
        """Constant forward speed, angular velocity weaves side to side -
        traces a wavy/serpentine line instead of a closed loop."""
        v = self.curvy_linear
        w = self.curvy_amplitude * math.sin(2 * math.pi * t / self.curvy_period)
        return v, w

    def _spiral(self, t):
        """Constant forward speed, angular velocity shrinks over each
        cycle so the turning radius (v/omega) grows - traces an
        outward spiral, then resets and spirals out again."""
        v = self.spiral_linear
        cycle_t = t % self.spiral_duration
        frac = cycle_t / self.spiral_duration  # 0 -> 1 over the cycle
        omega = (self.spiral_omega_start
                  - frac * (self.spiral_omega_start - self.spiral_omega_min))
        return v, max(omega, self.spiral_omega_min)

    def tick(self):
        t = self._elapsed()
        v, w = self._trajectory_fn(t)
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StudentTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop the robot
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
