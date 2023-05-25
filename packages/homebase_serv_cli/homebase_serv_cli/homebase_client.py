import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from threading import Thread

from custom_action_msgs.action import Homebase

import time


class HomebaseClient(Node, Thread):

    def __init__(self, id, signal, input):
        Node.__init__(self, 'homebase_client')
        Thread.__init__(self)
        
        self._action_client = ActionClient(self, Homebase, 'homebase')
        self.id = id

        self.signal = signal
        self.feedback = None
        self.input = input

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.daemon = True


    def run(self):
        self.send_goal(self.input)
        rclpy.spin(self)
    
    def timer_callback(self):
        if self.feedback is not None:
            for i, p in enumerate(self.feedback):
                self.signal.emit(str(self.id)+' DRON ' + str(i) + ' ' + str(p))

    def send_goal(self, order):
        goal_msg = Homebase.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        # Cuando se manda la goal se define el callback para el feedback
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, self.feedback_callback)

        # Despues se especifica el callback para el resultado
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Goal aceptada o rechazada
    def goal_response_callback(self, future):
        goal_handle = future.result()

        # Si rechazada, return
        if not goal_handle.accepted:
            self.signal.emit(str(self.id)+' Goal rejected :(')
            return

        # Si aceptada, informar y pedir resultado
        self.signal.emit(str(self.id)+' Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Callback para cuando llegue el resultado
    def get_result_callback(self, future):
        result = future.result().result
        self.signal.emit(str(self.id)+' Result: ' + result.result)

        self.signal.emit(str(self.id)+' HomebaseClient destroyed\n')
        self.destroy_node()

    # Callback para cuando llegue el feedback
    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback.distance

    def cancel_goal(self):
        self._action_client.wait_for_server()
        self._cancel_goal_future = self._action_client._cancel_goal_async(self._send_goal_future.result())
        self._cancel_goal_future.add_done_callback(self.goal_canceled_callback)

    def goal_canceled_callback(self, future):
        if future.result().return_code == 1 or future.result().return_code == 0:
            self.signal.emit(str(self.id)+' Goal canceled\n')
        else:
            self.signal.emit(str(self.id)+' Goal failed to cancel\n')


def main(args=None):
    rclpy.init(args=args)

    action_client = HomebaseClient()

    order = input('Introduce una orden para el swarm \n')

    future = action_client.send_goal(order)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()