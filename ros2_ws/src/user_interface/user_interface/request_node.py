import rclpy
from rclpy.node import Node
from rclpy import qos

from threading import Thread
from mutac_msgs.msg import UserRequest, Identifier, Alarm
from std_msgs.msg import String

from time import sleep, time

class RequestNode(Node, Thread):
    """Creates a node in a new thread. The node is used to send user requests to the monitoring system."""
    def __init__(self, id, signal, input):
        Node.__init__(self, 'info_node')
        Thread.__init__(self)

        self.id = id
        self.signal = signal
        self.input = input

        self.drone_responses = []

        self.drone_alarm = self.create_publisher(Alarm, '/planner/signal/drone_alarm', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))
        self.user_request = self.create_publisher(UserRequest, '/planner/comms/user_request', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.drone_response = self.create_subscription(String, '/planner/comms/drone_response', self.responseCallback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))


    def run(self):
        self.pub_request(self.input)

    def responseCallback(self, msg):
        """Callback function for the drone response subscriber. It prints the response in 
        the corresponding action tab."""
        self.get_logger().info('Response received at '+str(time()))
        self.drone_responses.append(msg.data)
        self.signal.emit(str(self.id)+' Response received\n')

    def pub_request(self, input):
        """Publishes the user request received as input to the user_request topic."""
        numeric_data = '-1'
        for s in input:
            if s.replace('.', '').isnumeric():
                numeric_data = s

        if 'state' in input:
            numeric_data = int(numeric_data)
            msg = UserRequest(identifier=Identifier(natural=numeric_data), text=String(data='state'))
            self.user_request.publish(msg)
            self.signal.emit(str(self.id)+' State request sent successfully. Waiting for responses...\n')

        elif 'cancel' in input:
            msg = UserRequest(identifier=Identifier(natural=-1), text=String(data='cancel'))
            self.user_request.publish(msg)
            self.signal.emit(str(self.id)+' Cancel request sent successfully. Waiting for responses...\n')

        elif 'height' in input:
            msg = UserRequest(identifier=Identifier(natural=-1), text=String(data='height '+numeric_data))
            self.user_request.publish(msg)
            self.signal.emit(str(self.id)+' Inspection height modification request sent successfully. Waiting for responses...\n')

        elif 'repeat' in input:
            numeric_data = int(numeric_data)
            msg = Alarm(identifier=Identifier(natural=numeric_data), alarm=1)
            self.drone_alarm.publish(msg)
            self.signal.emit(str(self.id)+' Repeat waypoint request sent successfully.\n')

            self.destroy_node()
            return

        else:
            self.signal.emit(str(self.id)+' Invalid input\n')

            self.destroy_node()
            return
        
        sleep(0.5)

        # prints the responses
        for response in self.drone_responses:
            self.signal.emit(str(self.id)+' DRON ' + str(response))

        self.destroy_node()
