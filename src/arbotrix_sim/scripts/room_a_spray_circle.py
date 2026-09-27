#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SprayCircle(Node):

    def __init__(self):
        super().__init__('spray_circle')

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.pump_pub = self.create_publisher(
            Bool, '/pump_cmd', 10)

        self.spray_completed_pub = self.create_publisher(
            Bool, '/spray_completed', 10)

        # Small circular pattern
        self.radius = 1.0

        # Slow spraying speed
        self.linear_speed = 0.15

        # v = r * omega
        self.angular_speed = self.linear_speed / self.radius

        # One complete revolution
        self.duration = (
            2.0 * math.pi
        ) / self.angular_speed

        self.start_time = None

    def pump(self, state):
        msg = Bool()
        msg.data = state
        self.pump_pub.publish(msg)

    def spray_completed(self):
        msg = Bool()
        msg.data = True
        self.spray_completed_pub.publish(msg)

    def velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        for _ in range(10):
            self.velocity(0.0, 0.0)
            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

    def run(self):

        self.get_logger().info(
            'Starting spray circle test.'
        )

        # -------------------------
        # PUMP ON
        # -------------------------

        self.pump(True)

        self.get_logger().info(
            'PUMP ON'
        )

        # -------------------------
        # ONE SECOND DELAY
        # -------------------------

        self.start_time = (
            self.get_clock().now()
        )

        while True:

            elapsed = (
                self.get_clock().now()
                - self.start_time
            ).nanoseconds / 1e9

            if elapsed >= 1.0:
                break

            self.velocity(0.0, 0.0)

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        # -------------------------
        # CIRCULAR MOVEMENT
        # -------------------------

        self.get_logger().info(
            f'Circle radius: {self.radius:.2f} m'
        )

        self.get_logger().info(
            f'Linear speed: {self.linear_speed:.2f} m/s'
        )

        self.get_logger().info(
            f'Angular speed: {self.angular_speed:.2f} rad/s'
        )

        self.get_logger().info(
            f'Circle duration: '
            f'{self.duration:.2f} simulated seconds'
        )

        self.start_time = (
            self.get_clock().now()
        )

        while True:

            elapsed = (
                self.get_clock().now()
                - self.start_time
            ).nanoseconds / 1e9

            if elapsed >= self.duration:
                break

            self.velocity(
                self.linear_speed,
                self.angular_speed
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        # -------------------------
        # STOP ROBOT
        # -------------------------

        self.stop()

        self.get_logger().info(
            'Circular pattern complete.'
        )

        # -------------------------
        # PUMP OFF
        # -------------------------

        self.pump(False)

        self.get_logger().info(
            'PUMP OFF'
        )

        # -------------------------
        # REPORT TRIGGER
        # -------------------------

        self.spray_completed()

        self.get_logger().info(
            'SPRAY COMPLETED SIGNAL SENT.'
        )

        self.get_logger().info(
            'Spray routine complete.'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SprayCircle()

    try:
        node.run()

    finally:

        node.stop()

        node.pump(False)

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
