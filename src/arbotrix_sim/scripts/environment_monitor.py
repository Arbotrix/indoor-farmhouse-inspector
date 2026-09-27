#!/usr/bin/env python3

import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class EnvironmentMonitor(Node):

    def __init__(self):
        super().__init__('environment_monitor')

        # ==========================================
        # ROOM B INSPECTION COORDINATE
        # ==========================================

        self.room_b_x = -8.5
        self.room_b_y = 5.2

        # Robot must come within this distance
        # of the Room B inspection coordinate.
        self.room_b_tolerance = 0.35

        # ==========================================
        # SENSOR VALUES
        # ==========================================

        self.room_a_temperature = None
        self.room_a_humidity = None

        self.room_b_temperature = None
        self.room_b_humidity = None

        # ==========================================
        # REPORT STATES
        # ==========================================

        self.room_a_report_generated = False
        self.room_b_report_generated = False

        self.room_a_sprayed = False

        # ==========================================
        # ROOM A TEMPERATURE
        # ==========================================

        self.room_a_temp_sub = self.create_subscription(
            Float64,
            '/room_a_sensor/temperature',
            self.room_a_temperature_callback,
            10
        )

        # ==========================================
        # ROOM A HUMIDITY
        # ==========================================

        self.room_a_humidity_sub = self.create_subscription(
            Float64,
            '/room_a_sensor/humidity',
            self.room_a_humidity_callback,
            10
        )

        # ==========================================
        # ROOM B TEMPERATURE
        # ==========================================

        self.room_b_temp_sub = self.create_subscription(
            Float64,
            '/room_b_sensor/temperature',
            self.room_b_temperature_callback,
            10
        )

        # ==========================================
        # ROOM B HUMIDITY
        # ==========================================

        self.room_b_humidity_sub = self.create_subscription(
            Float64,
            '/room_b_sensor/humidity',
            self.room_b_humidity_callback,
            10
        )

        # ==========================================
        # SPRAY COMPLETION
        # ==========================================

        self.spray_completed_sub = self.create_subscription(
            Bool,
            '/spray_completed',
            self.spray_completed_callback,
            10
        )

        # ==========================================
        # ROBOT ODOMETRY
        # ==========================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ==========================================
        # RVIZ WARNING PUBLISHER
        # ==========================================

        self.warning_pub = self.create_publisher(
            Marker,
            '/inspection_warning',
            10
        )

        self.get_logger().info(
            'Environment monitor started.'
        )

        self.get_logger().info(
            f'Room B inspection target: '
            f'({self.room_b_x:.2f}, {self.room_b_y:.2f})'
        )

    # ==========================================
    # SENSOR CALLBACKS
    # ==========================================

    def room_a_temperature_callback(self, msg):
        self.room_a_temperature = msg.data

    def room_a_humidity_callback(self, msg):
        self.room_a_humidity = msg.data

    def room_b_temperature_callback(self, msg):
        self.room_b_temperature = msg.data

    def room_b_humidity_callback(self, msg):
        self.room_b_humidity = msg.data

    # ==========================================
    # SPRAY COMPLETED CALLBACK
    # ==========================================

    def spray_completed_callback(self, msg):

        if msg.data and not self.room_a_report_generated:

            self.room_a_sprayed = True

            self.get_logger().info(
                'Spraying completed. Generating Room A report.'
            )

            self.generate_room_a_report()

    # ==========================================
    # ODOMETRY CALLBACK
    # ==========================================

    def odom_callback(self, msg):

        if self.room_b_report_generated:
            return

        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y

        distance = (
            (robot_x - self.room_b_x) ** 2
            +
            (robot_y - self.room_b_y) ** 2
        ) ** 0.5

        if distance <= self.room_b_tolerance:

            self.get_logger().info(
                f'Room B inspection coordinate reached. '
                f'Robot position: '
                f'({robot_x:.2f}, {robot_y:.2f})'
            )

            self.generate_room_b_report()

    # ==========================================
    # ROOM A REPORT
    # ==========================================

    def generate_room_a_report(self):

        if self.room_a_temperature is None:
            self.get_logger().warning(
                'Room A temperature has not been received yet.'
            )
            return

        if self.room_a_humidity is None:
            self.get_logger().warning(
                'Room A humidity has not been received yet.'
            )
            return

        # Temperature threshold
        if self.room_a_temperature < 20.0:
            temperature_status = 'LOW TEMPERATURE WARNING'
        else:
            temperature_status = 'NORMAL'

        # Humidity threshold
        if self.room_a_humidity < 40.0:
            humidity_status = 'LOW HUMIDITY WARNING'
        else:
            humidity_status = 'NORMAL'

        # Overall status
        if (
            temperature_status != 'NORMAL'
            or humidity_status != 'NORMAL'
        ):
            overall_status = 'WARNING'
        else:
            overall_status = 'NORMAL'

        # Reports directory
        report_directory = os.path.expanduser(
            '~/arbotrix_ws/reports'
        )

        os.makedirs(
            report_directory,
            exist_ok=True
        )

        report_path = os.path.join(
            report_directory,
            'room_a_report.txt'
        )

        current_time = datetime.now().strftime(
            '%Y-%m-%d %H:%M:%S'
        )

        report = f"""
========================================
INDOOR FARMHOUSE INSPECTOR
ROOM A INSPECTION REPORT
========================================

Inspection Time:
{current_time}

ENVIRONMENTAL READINGS

Temperature:
{self.room_a_temperature:.1f} °C

Temperature Status:
{temperature_status}

Humidity:
{self.room_a_humidity:.1f} %

Humidity Status:
{humidity_status}

SPRAYING

Water Sprayed:
YES

Spraying Status:
COMPLETED

OVERALL INSPECTION STATUS

{overall_status}

========================================
"""

        with open(
            report_path,
            'w'
        ) as report_file:

            report_file.write(report)

        self.room_a_report_generated = True

        self.get_logger().info(
            f'Room A report generated: {report_path}'
        )

        self.get_logger().info(
            f'Overall Room A status: {overall_status}'
        )

    # ==========================================
    # ROOM B REPORT
    # ==========================================

    def generate_room_b_report(self):

        if self.room_b_temperature is None:
            self.get_logger().warning(
                'Room B temperature has not been received yet.'
            )
            return

        if self.room_b_humidity is None:
            self.get_logger().warning(
                'Room B humidity has not been received yet.'
            )
            return

        # Room B temperature
        if self.room_b_temperature < 20.0:
            temperature_status = 'LOW TEMPERATURE WARNING'
        else:
            temperature_status = 'NORMAL'

        # Room B humidity
        if self.room_b_humidity < 40.0:
            humidity_status = 'LOW HUMIDITY WARNING'
        else:
            humidity_status = 'NORMAL'

        # For Room B, low temperature is the
        # warning we want to display in RViz.
        if temperature_status != 'NORMAL':
            overall_status = 'WARNING'
        else:
            overall_status = 'NORMAL'

        # Reports directory
        report_directory = os.path.expanduser(
            '~/arbotrix_ws/reports'
        )

        os.makedirs(
            report_directory,
            exist_ok=True
        )

        report_path = os.path.join(
            report_directory,
            'room_b_report.txt'
        )

        current_time = datetime.now().strftime(
            '%Y-%m-%d %H:%M:%S'
        )

        report = f"""
========================================
INDOOR FARMHOUSE INSPECTOR
ROOM B INSPECTION REPORT
========================================

Inspection Time:
{current_time}

INSPECTION COORDINATE

X:
{self.room_b_x:.2f}

Y:
{self.room_b_y:.2f}

ENVIRONMENTAL READINGS

Temperature:
{self.room_b_temperature:.1f} °C

Temperature Status:
{temperature_status}

Humidity:
{self.room_b_humidity:.1f} %

Humidity Status:
{humidity_status}

SPRAYING

Water Sprayed:
NO

Spraying Status:
NOT REQUIRED

OVERALL INSPECTION STATUS

{overall_status}

========================================
"""

        with open(
            report_path,
            'w'
        ) as report_file:

            report_file.write(report)

        self.room_b_report_generated = True

        self.get_logger().info(
            f'Room B report generated: {report_path}'
        )

        self.get_logger().info(
            f'Overall Room B status: {overall_status}'
        )

        # ======================================
        # LOW TEMPERATURE RVIZ WARNING
        # ======================================

        if self.room_b_temperature < 20.0:

            self.publish_low_temperature_warning(
                self.room_b_temperature
            )

    # ==========================================
    # RVIZ LOW TEMPERATURE WARNING
    # ==========================================

    def publish_low_temperature_warning(
        self,
        temperature
    ):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'environment_warning'
        marker.id = 1

        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Warning appears near Room B sensor
        marker.pose.position.x = -8.5
        marker.pose.position.y = 6.3
        marker.pose.position.z = 1.8

        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.55

        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.color.a = 1.0

        marker.text = (
            f'LOW TEMPERATURE WARNING\n'
            f'{temperature:.1f} C'
        )

        marker.lifetime.sec = 0

        self.warning_pub.publish(marker)

        self.get_logger().info(
            'RViz low-temperature warning published.'
        )


def main(args=None):

    rclpy.init(args=args)

    node = EnvironmentMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()