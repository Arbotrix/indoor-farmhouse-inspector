import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from enum import Enum


class RobotState(Enum):
    IDLE = 1
    MOVING = 2
    WORKING = 3
    ERROR = 4


class FleetManager(Node):

    def __init__(self):
        super().__init__('fleet_manager')

        self.state = RobotState.IDLE

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.goal_sent = False
        self.working_start_time = None

        self.dispatch_srv = self.create_service(
            Trigger,
            'dispatch_order',
            self.dispatch_callback
        )

        self.timer = self.create_timer(
            0.2,
            self.control_loop
        )

        self.get_logger().info(
            'Fleet Manager started. State: IDLE'
        )

    def dispatch_callback(self, request, response):

        if self.state == RobotState.IDLE:

            self.state = RobotState.MOVING
            self.goal_sent = False

            self.get_logger().info(
                'Dispatch received. Transitioning to MOVING.'
            )

            response.success = True
            response.message = 'Mission started.'

        else:

            response.success = False
            response.message = 'Robot is busy.'

        return response

    def control_loop(self):

        if self.state == RobotState.IDLE:
            pass

        elif self.state == RobotState.MOVING:

            if not self.goal_sent:
                self.send_nav_goal()

        elif self.state == RobotState.WORKING:

            if self.working_start_time is None:
                self.working_start_time = self.get_clock().now()

                self.get_logger().info(
                    'WORKING: Simulating payload operation for 5 seconds.'
                )

            elapsed = (
                self.get_clock().now() - self.working_start_time
            ).nanoseconds / 1e9

            if elapsed >= 5.0:
                self.get_logger().info(
                    'WORKING complete. Returning to IDLE.'
                )

                self.state = RobotState.IDLE
                self.working_start_time = None

        elif self.state == RobotState.ERROR:

            self.get_logger().error(
                'System in ERROR state.'
            )

    def send_nav_goal(self):

        self.goal_sent = True

        self.get_logger().info(
            'Waiting for Nav2 action server...'
        )

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Nav2 action server not available.'
            )

            self.state = RobotState.ERROR
            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 2.0
        goal_msg.pose.pose.position.y = 2.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            'Sending navigation goal: X=2.0, Y=2.0'
        )

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def feedback_callback(self, feedback_msg):

        distance = feedback_msg.feedback.distance_remaining

        self.get_logger().info(
            f'Distance remaining: {distance:.2f} m'
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Navigation goal rejected.'
            )

            self.state = RobotState.ERROR
            return

        self.get_logger().info(
            'Navigation goal accepted.'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.navigation_result_callback
        )

    def navigation_result_callback(self, future):

        status = future.result().status

        self.get_logger().info(
            f'Navigation finished with result code: {status}'
        )

        if status == 4:

            self.get_logger().info(
                'Navigation successful. Transitioning to WORKING.'
            )

            self.state = RobotState.WORKING
            self.working_start_time = None

        else:

            self.get_logger().error(
                'Navigation failed.'
            )

            self.state = RobotState.ERROR


def main(args=None):

    rclpy.init(args=args)

    node = FleetManager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    