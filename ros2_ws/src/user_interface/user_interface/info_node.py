import rclpy
from rclpy.node import Node
from rclpy import qos

from threading import Thread
from mutac_msgs.msg import UserRequest, Identifier
from std_msgs.msg import String

from time import sleep

class InfoNode(Node, Thread):
    def __init__(self, id, signal, input):
        Node.__init__(self, 'info_node')
        Thread.__init__(self)

        self.id = id
        self.signal = signal
        self.input = input

        self.drone_states = []

        self.user_request = self.create_publisher(UserRequest, '/planner/comms/user_request', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.drone_response = self.create_subscription(String, '/planner/comms/drone_response', self.responseCallback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))


    def run(self):
        self.pub_request(self.input)

    def responseCallback(self, msg):
        self.get_logger().info('Response received')
        self.drone_states.append(msg.data)
        self.signal.emit(str(self.id)+' Response received\n')

    def pub_request(self, input):
        if input == 'swarm_state':
            msg = UserRequest(identifier=Identifier(natural=-1), text=String(data=input))
            self.user_request.publish(msg)
            self.signal.emit(str(self.id)+' Request sent successfully. Waiting for responses...\n')

            sleep(0.5)

            # print the responses
            for state in self.drone_states:
                self.signal.emit(str(self.id)+' DRON ' + str(state))

        else:
            self.signal.emit(str(self.id)+' Invalid input\n')

        self.destroy_node()
