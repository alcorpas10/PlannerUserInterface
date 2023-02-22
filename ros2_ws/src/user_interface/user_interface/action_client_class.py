import rclpy

from threading import Thread


class ActionClientClass(Thread):

    def __init__(self, action_client):
        Thread.__init__(self)
        self.action_client = action_client

    def run(self, input):
        self.action_client.send_goal(input)

        rclpy.spin(self.action_client)