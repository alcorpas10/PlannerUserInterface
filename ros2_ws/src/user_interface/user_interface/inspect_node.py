from rclpy.node import Node

from threading import Thread
from mutac_msgs.srv import GeneratePlan

class InspectNode(Node, Thread):
    def __init__(self, id, signal, plan):
        Node.__init__(self, 'interface_node')
        Thread.__init__(self)

        self.id = id
        self.signal = signal
        self.plan = plan

        self.plan_client = self.create_client(GeneratePlan, '/planner/generate_plan')


    def run(self):
        self.pub_plan(self.plan)

    def pub_plan(self, plan):
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().warn('Service not available, waiting again...')
            self.signal.emit(str(self.id)+' Service not available, waiting again...\n')

        self.plan_client.call_async(plan)
        self.signal.emit(str(self.id)+' Plan sent successfully\n')

        self.destroy_node()
