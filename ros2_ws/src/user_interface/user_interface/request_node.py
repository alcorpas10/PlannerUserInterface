import rclpy
from rclpy.node import Node
from rclpy import qos

from threading import Thread
from mutac_msgs.msg import UserRequest, Identifier
from std_msgs.msg import String

from time import sleep

class RequestNode(Node, Thread):
    def __init__(self, id, signal, input):
        Node.__init__(self, 'info_node')
        Thread.__init__(self)

        self.id = id
        self.signal = signal
        self.input = input

        self.drone_responses = []

        self.user_request = self.create_publisher(UserRequest, '/planner/comms/user_request', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.drone_response = self.create_subscription(String, '/planner/comms/drone_response', self.responseCallback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))


    def run(self):
        self.pub_request(self.input)

    def responseCallback(self, msg):
        self.get_logger().info('Response received')
        self.drone_responses.append(msg.data)
        self.signal.emit(str(self.id)+' Response received\n')

    def pub_request(self, input):
        if input == 'swarm_state':
            msg = UserRequest(identifier=Identifier(natural=-1), text=String(data=input))
            self.user_request.publish(msg)
            self.signal.emit(str(self.id)+' State request sent successfully. Waiting for responses...\n')

            sleep(0.5)

            # print the responses
            for response in self.drone_responses:
                self.signal.emit(str(self.id)+' DRON ' + str(response))
        elif input == 'cancel_mission':
            msg = UserRequest(Identifier=Identifier(natural=-1), text=String(data=input))
            self.user_request.publish(msg)
            self.signal.emit(str(self.id)+' Cancel request sent successfully. Waiting for responses...\n')

            sleep(0.5)

            # print the responses
            for response in self.drone_responses:
                self.signal.emit(str(self.id)+' DRON ' + str(response))            
        else:
            self.signal.emit(str(self.id)+' Invalid input\n')

        self.destroy_node()
