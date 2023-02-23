import rclpy

from threading import Thread


class ActionClientClass(Thread):

    def __init__(self, action_client, input):
        Thread.__init__(self)
        self.action_client = action_client
        self.input = input

    def run(self):
        self.action_client.send_goal(self.input)

        rclpy.spin(self.action_client)