#!/usr/bin/env python3
"""
A fake robot simulator that publishes the ground truth odometry (/odom), a
noisy+drifting odometry (/odom1), and noisy GPS measurements (/fake_gps) 
based on the ground truth.
"""

import math
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FakeGroundTruthAndGPS(Node):
    def __init__(self):
        super().__init__('fake_odometry_and_gps')

        self.declare_parameter('sim_dt', 0.02)
        self.declare_parameter('position_noise_std', 0.3)
        self.declare_parameter('orientation_noise_std', 0.02)
        self.declare_parameter('velocity_noise_std', 0.01)
        self.declare_parameter('angular_velocity_noise_std', 0.005)
        self.declare_parameter('position_drift_rate', [0.05, 0.01, 0.0])
        self.declare_parameter('orientation_drift_rate', [0.0, 0.0, 0.0005])
        
        self.declare_parameter('gps_position_noise_std', 0.1)
        self.declare_parameter('gps_rate', 1.0)

        self.sim_dt = self.get_parameter('sim_dt').value
        self.position_noise_std = self.get_parameter('position_noise_std').value
        self.orientation_noise_std = self.get_parameter('orientation_noise_std').value
        self.velocity_noise_std = self.get_parameter('velocity_noise_std').value
        self.angular_velocity_noise_std = self.get_parameter(
            'angular_velocity_noise_std').value
        self.position_drift_rate = np.array(
            self.get_parameter('position_drift_rate').value)
        self.orientation_drift_rate = np.array(
            self.get_parameter('orientation_drift_rate').value)
        self.gps_position_noise_std = self.get_parameter(
            'gps_position_noise_std').value
        self.gps_rate = self.get_parameter('gps_rate').value

        self.position_drift_acc = np.zeros(3)
        self.orientation_drift_acc = np.zeros(3)

        self.vx = 0.0
        self.wz = 0.0
        self.pose = np.zeros(3)  # ground truth [x, y, yaw]

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.odom1_pub = self.create_publisher(Odometry, '/odom1', 10)
        self.gps_pub = self.create_publisher(Odometry, '/fake_gps', 10)

        self.create_timer(self.sim_dt, self.step)
        self._gps_period = 1.0 / self.gps_rate
        self._time_since_gps = 0.0

        self.get_logger().info(
            'Fake ground truth + GPS started (Gazebo-free stand-in for '
            'fake_odom.py). Publishing /odom (ground truth), /odom1 '
            '(noisy+drifting), and /fake_gps.')

    def cmd_vel_cb(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def step(self):
        dt = self.sim_dt
        yaw = self.pose[2]
        self.pose[0] += dt * self.vx * math.cos(yaw)
        self.pose[1] += dt * self.vx * math.sin(yaw)
        self.pose[2] += dt * self.wz

        stamp = self.get_clock().now().to_msg()
        gt_qx, gt_qy, gt_qz, gt_qw = quat_from_yaw(self.pose[2])

        # /odom: ground truth
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = float(self.pose[0])
        odom.pose.pose.position.y = float(self.pose[1])
        odom.pose.pose.orientation.x = gt_qx
        odom.pose.pose.orientation.y = gt_qy
        odom.pose.pose.orientation.z = gt_qz
        odom.pose.pose.orientation.w = gt_qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        # /odom1: noisy + drifting
        self.position_drift_acc += self.position_drift_rate * dt
        self.orientation_drift_acc += self.orientation_drift_rate * dt

        noisy_yaw = (self.pose[2]
                     + np.random.normal(0, self.orientation_noise_std)
                     + self.orientation_drift_acc[2])
        n_qx, n_qy, n_qz, n_qw = quat_from_yaw(noisy_yaw)

        odom1 = Odometry()
        odom1.header.stamp = stamp
        odom1.header.frame_id = 'odom'
        odom1.pose.pose.position.x = float(
            self.pose[0] + np.random.normal(0, self.position_noise_std)
            + self.position_drift_acc[0])
        odom1.pose.pose.position.y = float(
            self.pose[1] + np.random.normal(0, self.position_noise_std)
            + self.position_drift_acc[1])
        odom1.pose.pose.orientation.x = n_qx
        odom1.pose.pose.orientation.y = n_qy
        odom1.pose.pose.orientation.z = n_qz
        odom1.pose.pose.orientation.w = n_qw
        odom1.twist.twist.linear.x = self.vx + np.random.normal(
            0, self.velocity_noise_std)
        odom1.twist.twist.angular.z = self.wz + np.random.normal(
            0, self.angular_velocity_noise_std)
        self.odom1_pub.publish(odom1)

        # /fake_gps: rate-limited, noisy ground truth
        self._time_since_gps += dt
        if self._time_since_gps >= self._gps_period:
            self._time_since_gps = 0.0
            gps = Odometry()
            gps.header.stamp = stamp
            gps.header.frame_id = 'odom'
            gps.pose.pose.position.x = float(
                self.pose[0] + np.random.normal(0, self.gps_position_noise_std))
            gps.pose.pose.position.y = float(
                self.pose[1] + np.random.normal(0, self.gps_position_noise_std))
            gps.pose.pose.orientation.x = gt_qx
            gps.pose.pose.orientation.y = gt_qy
            gps.pose.pose.orientation.z = gt_qz
            gps.pose.pose.orientation.w = gt_qw
            gps.twist.twist.linear.x = self.vx
            gps.twist.twist.angular.z = self.wz
            self.gps_pub.publish(gps)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGroundTruthAndGPS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
