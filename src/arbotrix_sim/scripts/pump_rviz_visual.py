import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class PumpRVizVisual(Node):

    def __init__(self):
        super().__init__('pump_rviz_visual')

        self.pump_on = False

        self.subscription = self.create_subscription(
            Bool,
            '/pump_state',
            self.pump_state_callback,
            10
        )

        self.publisher = self.create_publisher(
            MarkerArray,
            '/pump_spray_markers',
            10
        )

        self.timer = self.create_timer(
            0.2,
            self.publish_markers
        )

    def pump_state_callback(self, msg):
        self.pump_on = msg.data

    def create_marker(
        self,
        marker_id,
        marker_type,
        x,
        y,
        z,
        scale_x,
        scale_y,
        scale_z
    ):

        marker = Marker()

        # Marker is attached to the robot
        marker.header.frame_id = 'base_link'

        # Use latest available transform
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0

        # Keep marker attached to base_link
        marker.frame_locked = True

        marker.ns = 'water_spray'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z

        marker.color.r = 0.05
        marker.color.g = 0.55
        marker.color.b = 1.0
        marker.color.a = 0.8

        return marker

    def publish_markers(self):

        marker_array = MarkerArray()

        if self.pump_on:

            # Main spray stream
            spray = self.create_marker(
                0,
                Marker.CYLINDER,
                0.645,
                0.0,
                0.29,
                0.07,
                0.07,
                0.50
            )

            # Water droplets
            droplet_1 = self.create_marker(
                1,
                Marker.SPHERE,
                0.92,
                0.025,
                0.32,
                0.07,
                0.07,
                0.07
            )

            droplet_2 = self.create_marker(
                2,
                Marker.SPHERE,
                1.02,
                -0.025,
                0.27,
                0.056,
                0.056,
                0.056
            )

            droplet_3 = self.create_marker(
                3,
                Marker.SPHERE,
                1.12,
                0.02,
                0.315,
                0.064,
                0.064,
                0.064
            )

            droplet_4 = self.create_marker(
                4,
                Marker.SPHERE,
                1.22,
                -0.02,
                0.275,
                0.05,
                0.05,
                0.05
            )

            # Rotate spray cylinder to point forward
            spray.pose.orientation.x = 0.0
            spray.pose.orientation.y = 0.7071068
            spray.pose.orientation.z = 0.0
            spray.pose.orientation.w = 0.7071068

            marker_array.markers.extend([
                spray,
                droplet_1,
                droplet_2,
                droplet_3,
                droplet_4
            ])

        else:

            # Delete all spray markers when pump is OFF
            for marker_id in range(5):

                marker = Marker()

                marker.header.frame_id = 'base_link'

                # Use latest transform
                marker.header.stamp.sec = 0
                marker.header.stamp.nanosec = 0

                marker.frame_locked = True

                marker.ns = 'water_spray'
                marker.id = marker_id
                marker.action = Marker.DELETE

                marker_array.markers.append(marker)

        self.publisher.publish(marker_array)


def main(args=None):

    rclpy.init(args=args)

    node = PumpRVizVisual()

    # Use Gazebo simulation time
    node.set_parameters([
        rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
    ])

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()