import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from threading import Thread

from custom_action_msgs.action import PointAction
from geometry_msgs.msg import Point

import numpy as np
import cv2


class PointActionClient(Node, Thread):

    def __init__(self, id, signal_str, signal_img, input):
        Node.__init__(self, 'point_action_client')
        Thread.__init__(self)
        self._action_client = ActionClient(self, PointAction, 'point_action')
        self.id = id

        self.signal_str = signal_str
        self.signal_img = signal_img
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
                self.signal_str.emit(str(self.id)+' DRON ' + str(i) + ' ' + str(p))

    def send_goal(self, order):
        goal_msg = PointAction.Goal()
        goal_msg.goal = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.signal_str.emit(str(self.id)+' Goal rejected :(\n')
            return

        self.signal_str.emit(str(self.id)+' Goal accepted :)\n')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        
        for i, msg in enumerate(result.result):
            img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img_data = cv2.cvtColor(img_data, cv2.COLOR_BGR2RGB)
            self.signal_str.emit(str(self.id)+' DRON ' + str(i) + ' ' + str(img_data.shape))
            self.signal_img.emit(img_data)

        self.signal_str.emit(str(self.id)+' PointActionClient destroyed\n')
        self.destroy_node()

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback.distance

    def cancel_goal(self):
        self._action_client.wait_for_server()
        self._cancel_goal_future = self._action_client._cancel_goal_async(self._send_goal_future.result())
        self._cancel_goal_future.add_done_callback(self.goal_canceled_callback)

    def goal_canceled_callback(self, future):
        if future.result().return_code == 1 or future.result().return_code == 0:
            self.signal_str.emit(str(self.id)+' Goal canceled\n')
        else:
            self.signal_str.emit(str(self.id)+' Goal failed to cancel\n')


def main(args=None):
    rclpy.init(args=args)

    action_client = PointActionClient()
    
    x = float(input("Insert X coordinate: "))
    y = float(input("Insert Y coordinate: "))
    z = float(input("Insert Z coordinate: "))

    action_client.send_goal(Point(x=x, y=y, z=z))

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
