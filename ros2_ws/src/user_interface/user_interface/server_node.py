from rclpy.node import Node

from threading import Thread
from mutac_msgs.srv import DroneRequest
from mutac_msgs.msg import DroneComms

class ServerNode(Node, Thread):
    def __init__(self, id, signal, executor):
        Node.__init__(self, 'server_node')
        Thread.__init__(self)

        self.id = id
        self.signal = signal
        self.executor = executor

        self.drone_request_server = self.create_service(DroneRequest, '/planner/comms/drone_request', self.droneRequestCallback)


    def run(self):
        self.executor.spin()

    def droneRequestCallback(self, request, response):
        self.get_logger().info('Drone request received')
        if request.type == DroneComms.CANCEL:
            self.signal.emit(str(self.id)+' Cancel request received\n')
        else:
            self.signal.emit(str(self.id)+' Unknown request received\n')

        return response