import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class HomebaseClient(Node):

    def __init__(self):
        super().__init__('homebase_action_client')
        self._action_client = ActionClient(self, Homebase, 'homebase')

    def send_goal(self, order):
        goal_msg = Homebase.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        # Cuando se manda la goal se define el callback para el feedback
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback)

        # Despues se especifica el callback para el resultado
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Goal aceptada o rechazada
    def goal_response_callback(self, future):
        goal_handle = future.result()
        # Si rechazada, return
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        # Si aceptada, informar y pedir resultado
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Callback para cuando llegue el resultado
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

    # Callback para cuando llegue el feedback
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        for p in feedback:
            self.get_logger().info(str(p))
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args)

    action_client = HomebaseClient()

    order = input('Introduce una orden para el swarm')

    future = action_client.send_goal(order)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()