#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose


class AutonomousInspection(Node):

    def __init__(self):
        super().__init__('autonomous_inspection')

        # ==========================================
        # ROUTE COORDINATES
        # ==========================================

        # Home / docking area
        self.home_x = 0.0
        self.home_y = 8.0

        # Room A inspection point
        self.room_a_x = 8.5
        self.room_a_y = 2.5

        # Room B inspection point
        self.room_b_x = -8.5
        self.room_b_y = 5.2

        # ==========================================
        # PUBLISHERS
        # ==========================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.pump_pub = self.create_publisher(
            Bool,
            '/pump_cmd',
            10
        )

        self.spray_completed_pub = self.create_publisher(
            Bool,
            '/spray_completed',
            10
        )

        # ==========================================
        # NAV2
        # ==========================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # ==========================================
        # ODOMETRY
        # ==========================================

        self.current_x = None
        self.current_y = None

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Autonomous inspection controller started.'
        )

    # ==========================================
    # ODOMETRY
    # ==========================================

    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    # ==========================================
    # PUMP CONTROL
    # ==========================================

    def pump(self, state):

        msg = Bool()
        msg.data = state

        self.pump_pub.publish(msg)

        if state:
            self.get_logger().info(
                'PUMP ON'
            )
        else:
            self.get_logger().info(
                'PUMP OFF'
            )

    # ==========================================
    # STOP ROBOT
    # ==========================================

    def stop_robot(self):

        msg = Twist()

        for _ in range(10):

            self.cmd_pub.publish(msg)

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

    # ==========================================
    # NAVIGATE TO COORDINATE
    # ==========================================

    def navigate_to(
        self,
        x,
        y,
        yaw=0.0
    ):

        self.get_logger().info(
            f'Navigating to '
            f'X={x:.2f}, Y={y:.2f}'
        )

        self.get_logger().info(
            'Waiting for Nav2...'
        )

        if not self.nav_client.wait_for_server(
            timeout_sec=15.0
        ):

            self.get_logger().error(
                'Nav2 NavigateToPose action server '
                'is not available.'
            )

            return False

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(
            yaw / 2.0
        )
        goal.pose.pose.orientation.w = math.cos(
            yaw / 2.0
        )

        send_goal_future = (
            self.nav_client.send_goal_async(
                goal
            )
        )

        while not send_goal_future.done():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        goal_handle = send_goal_future.result()

        if goal_handle is None:

            self.get_logger().error(
                'Could not obtain Nav2 goal handle.'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                'Nav2 rejected the navigation goal.'
            )

            return False

        self.get_logger().info(
            'Navigation goal accepted.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        while not result_future.done():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        result = result_future.result()

        if result is None:

            self.get_logger().error(
                'Nav2 returned no result.'
            )

            return False

        self.get_logger().info(
            f'Navigation finished. '
            f'Status = {result.status}'
        )

        # Nav2 SUCCESS status = 4
        if result.status == 4:

            self.get_logger().info(
                'Navigation goal reached.'
            )

            return True

        self.get_logger().error(
            'Navigation goal was not completed.'
        )

        return False

    # ==========================================
    # ROOM A SPRAY + CIRCLE
    # ==========================================

    def room_a_inspection(self):

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            'ROOM A INSPECTION START'
        )

        self.get_logger().info(
            '======================================'
        )

        # --------------------------------------
        # Pump ON
        # --------------------------------------

        self.pump(True)

        # Allow spray visualization/pump to start
        start_time = time.monotonic()

        while time.monotonic() - start_time < 1.0:

            self.stop_robot()

        # --------------------------------------
        # Circular inspection
        # --------------------------------------

        radius = 1.0
        linear_speed = 0.15

        angular_speed = (
            linear_speed / radius
        )

        duration = (
            2.0 * math.pi
        ) / angular_speed

        self.get_logger().info(
            f'Circle radius: '
            f'{radius:.2f} m'
        )

        self.get_logger().info(
            f'Circle duration: '
            f'{duration:.2f} seconds'
        )

        start_time = time.monotonic()

        while (
            time.monotonic() - start_time
            < duration
        ):

            velocity = Twist()

            velocity.linear.x = linear_speed
            velocity.angular.z = angular_speed

            self.cmd_pub.publish(
                velocity
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        # --------------------------------------
        # Stop
        # --------------------------------------

        self.stop_robot()

        # --------------------------------------
        # Pump OFF
        # --------------------------------------

        self.pump(False)

        # --------------------------------------
        # Tell environment monitor
        # --------------------------------------

        self.get_logger().info(
            'Sending spray completed signal.'
        )

        spray_msg = Bool()
        spray_msg.data = True

        # Send multiple messages so the monitor
        # reliably receives the signal.
        for _ in range(5):

            self.spray_completed_pub.publish(
                spray_msg
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        self.get_logger().info(
            'ROOM A INSPECTION COMPLETE'
        )

    # ==========================================
    # MAIN MISSION
    # ==========================================

    def run_mission(self):

        self.get_logger().info(
            ''
        )

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            'INDOOR FARMHOUSE INSPECTOR'
        )

        self.get_logger().info(
            'AUTONOMOUS MISSION START'
        )

        self.get_logger().info(
            '======================================'
        )

        # ======================================
        # HOME → ROOM A
        # ======================================

        self.get_logger().info(
            'STEP 1: HOME -> ROOM A'
        )

        success = self.navigate_to(
            self.room_a_x,
            self.room_a_y
        )

        if not success:

            self.get_logger().error(
                'Could not reach Room A.'
            )

            return

        # ======================================
        # ROOM A INSPECTION
        # ======================================

        self.get_logger().info(
            'STEP 2: ROOM A INSPECTION'
        )

        self.room_a_inspection()

        # Give environment monitor time
        # to create Room A report.
        time.sleep(2.0)

        # ======================================
        # ROOM A → ROOM B
        # ======================================

        self.get_logger().info(
            'STEP 3: ROOM A -> ROOM B'
        )

        success = self.navigate_to(
            self.room_b_x,
            self.room_b_y
        )

        if not success:

            self.get_logger().error(
                'Could not reach Room B.'
            )

            return

        # ======================================
        # ROOM B
        # ======================================

        self.get_logger().info(
            'STEP 4: ROOM B INSPECTION'
        )

        self.get_logger().info(
            f'Room B target reached at '
            f'({self.room_b_x:.2f}, '
            f'{self.room_b_y:.2f})'
        )

        # Environment monitor detects the
        # coordinate from /odom and generates
        # room_b_report.txt.

        # Give it time to generate report
        for _ in range(20):

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        # ======================================
        # ROOM B → HOME
        # ======================================

        self.get_logger().info(
            'STEP 5: ROOM B -> HOME'
        )

        success = self.navigate_to(
            self.home_x,
            self.home_y
        )

        if not success:

            self.get_logger().error(
                'Could not return to Home.'
            )

            return

        # ======================================
        # MISSION COMPLETE
        # ======================================

        self.stop_robot()

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            'AUTONOMOUS MISSION COMPLETE'
        )

        self.get_logger().info(
            'Robot returned to Home/Docking area.'
        )

        self.get_logger().info(
            '======================================'
        )


def main(args=None):

    rclpy.init(args=args)

    node = AutonomousInspection()

    try:

        node.run_mission()

    except KeyboardInterrupt:

        node.get_logger().warning(
            'Mission interrupted by user.'
        )

        node.stop_robot()

    finally:

        node.stop_robot()
        node.pump(False)

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()