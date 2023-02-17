import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from accion.action import PointAction
from geometry_msgs.msg import Point


class PointActionClient(Node):

    def __init__(self):
        super().__init__('point_action_client')
        self._action_client = ActionClient(self, PointAction, 'point_action')

    def send_goal(self, order):
        goal_msg = PointAction.Goal()
        goal_msg.goal = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Final point: {0}'.format(result.result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Distance traveled: {0}'.format(feedback.distance))


def main(args=None):
    rclpy.init(args=args)

    action_client = PointActionClient()
    
    x = float(input("Inserte variable X: "))
    y = float(input("Inserte variable Y: "))
    z = float(input("Inserte variable Z: "))

    action_client.send_goal(Point(x=x, y=y, z=z))

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()