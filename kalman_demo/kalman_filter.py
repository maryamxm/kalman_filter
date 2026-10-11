#!/usr/bin/env python3
"""
A simple Kalman filter node for fusing odometry and GPS measurements.

"""
import math
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SimpleKalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_simple')

        # Params
        self.declare_parameter('dt', 0.1)
        self.dt = self.get_parameter('dt').value

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/fake_gps', self.gps_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Odometry, '/kalman_estimate', 10)

        # Initial State: [x, y, yaw]
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1

        # Noise covariances (tuned values)
        self.Q = np.eye(3) * float(1e-2)  # Process noise
        self.R = np.eye(2) * float(1.9)   # GPS noise

        # Latest inputs/measurements - essential for the prediction step
        # received from /cmd_vel
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0

        # Latest GPS measurement
        self.gps = None  # 2x1: [x; y]

        # Timer for Kalman update
        self.create_timer(self.dt, self.update_kalman)

        self.get_logger().info('Kalman Filter start')

    def cmd_vel_callback(self, msg):
        """Store the latest cmd velocities."""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.yaw_rate = msg.angular.z

    def gps_callback(self, msg):
        """Store the latest GPS measurement."""
        self.gps = np.array([[msg.pose.pose.position.x],
                             [msg.pose.pose.position.y]])

    def update_kalman(self):
        
        dt = float(self.dt)  # time step

        Q_process = self.Q
        R_gps = self.R

        x_prev = self.x  # (3x1) matrix
        p_t = self.P

        theta = float(x_prev[2, 0])
        vx = float(self.vx)
        wz = float(self.yaw_rate)

        # (2x1) matrix: input from cmd_vel
        u_t = np.array([[vx],
                        [wz]])

        # --- Prediction ---

        # Control matrix
        B = np.array([
            [dt * np.cos(theta), 0.0],
            [dt * np.sin(theta), 0.0],
            [0.0,                 dt]])

        # System model: Jacobian
        A = np.array([
            [1.0, 0.0, -dt * vx * np.sin(theta)],
            [0.0, 1.0,  dt * vx * np.cos(theta)],
            [0.0, 0.0,  1.0]])

        x_pred = x_prev + B @ u_t
        P_pred = A @ (p_t @ A.T) + Q_process

        # --- Correction (GPS sensor) ---

        # Observation matrix
        C_t = np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])

        if self.gps is not None:

            z = self.gps
            y = z - (C_t @ x_pred)                  # measurement residual
            S = C_t @ P_pred @ C_t.T + R_gps         # residual covariance
            K = P_pred @ C_t.T @ np.linalg.inv(S)    # kalman gain

            x_upd = x_pred + K @ y
            I = np.eye(3)
            P_upd = (I - K @ C_t) @ P_pred

            self.x = x_upd
            self.P = P_upd

        else:
            self.x = x_pred
            self.P = P_pred

        self.publish_estimate()

    def publish_estimate(self):
        """Publish the current state estimate as an Odometry message."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])

        # yaw -> quaternion (roll = pitch = 0)
        yaw = float(self.x[2])
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.twist.twist.linear.x = float(self.vx)
        msg.twist.twist.linear.y = float(self.vy)
        msg.twist.twist.angular.z = float(self.yaw_rate)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleKalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
