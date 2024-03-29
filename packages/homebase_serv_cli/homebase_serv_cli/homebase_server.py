
import rclpy
import sys
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node

from custom_action_msgs.action import Homebase
from mutac_msgs.msg import Plan, LabeledPath, LabeledPoint, Label
from geometry_msgs.msg import Point
from mutac_msgs.msg import Metrics, Identifier
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from math import sqrt


class HomebaseActionServer(Node):

    def __init__(self, n_drones, callback_group):
        super().__init__('homebase_action_server')
        self._action_server = ActionServer(self, Homebase, 'homebase',
            self.execute_callback, callback_group=callback_group, cancel_callback=self.cancel_callback)
        
        self.n_drones = n_drones

        self.drone_points = [None for i in range(0,n_drones)]
        
        #self.accepted_commands = ['go_homebase', 'abort']

        #self.act_goal_handler = None

        self.desired_points = [self.nuevoPunto() for i in range(0,n_drones)]

        self.distances = [0 for i in range(0,n_drones)]

        self.path_publisher = self.create_publisher(Plan, '/mutac/planned_paths', 100, callback_group=callback_group)
        self.pos_listener = [self.create_subscription(Metrics, '/mutac/drone'+str(i)+'/performance_metrics', 
                            self.listener_callback, 100, callback_group=callback_group) for i in range(1, self.n_drones+1)]


    def nuevoPunto(self):
            p = Point()
            p._x = float(0)
            p._y = float(0)
            p._z = float(0)
            return p

    def listener_callback(self,msg):
        drone_id = msg.identifier.natural - 1
        self.drone_points[drone_id] = msg.position


    def execute_callback(self, goal_handle):
        print('------------------------------------')
        self.get_logger().info('Executing goal')
        print('------------------------------------')

        """if goal_handle.request.order not in self.accepted_commands:
            self.get_logger().info('Goal rejected')
            goal_handle.canceled()
            result = 'Goal ' + str(goal_handle.request.order) + ' not in available orders'
            return result

        self.set_action_params(goal_handle.request.order)"""
        self.desired_points = [self.nuevoPunto() for i in range(0,self.n_drones)]

        """if self.act_goal_handler is not None:
            self.get_logger().info('------------------------------------------------------------------------------------------------------------------')
            self.get_logger().info('Recibida la orden de ' + str(goal_handle.request.order) + ' mientras se completaba la actual, cancelando la actual')
            self.get_logger().info('------------------------------------------------------------------------------------------------------------------')
            abort_result = Homebase.Result()
            abort_result.result = 'Recibida la orden de ' + str(goal_handle.request.order) + ' mientras se completaba la actual, cancelando la actual'
            self.act_goal_handler.abort()

        self.act_goal_handler = goal_handle"""


        plan = Plan()
        plan.paths = self.generate_labeled_paths()
        self.path_publisher.publish(plan)
        self.get_logger().info('PLAN =  \n' + str(plan))

        self.get_logger().info('------------------------------------------')

        feedback_msg = Homebase.Feedback()

        # Publicar la posicion de cada dron mientras vuelven
        drones_in_desired = False
        while not drones_in_desired:
            # for i in range(0, self.n_drones):
                # self.get_logger().info('El dron ' + str(i) + ' esta en el punto ' + str(self.drone_points[i]))
            #if goal_handle.request.order == self.act_goal_handler.request.order:
            drones_in_desired = self.points_is_desired()
            feedback_msg.distance = self.generate_feedback_msg()
            goal_handle.publish_feedback(feedback_msg)
        
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()

            self.get_logger().info('Goal canceled')
            print('------------------------------------')

            result = Homebase.Result()
            result.result = 'Goal canceled'
            return result
        
        goal_handle.succeed()

        result = Homebase.Result()
        self.get_logger().info('Goal reached')
        print('------------------------------------')

        #self.act_goal_handler = None

        result.result = 'Drones en base'
        return result

    """def set_action_params(self, order):
        if order == 'go_homebase':
            self.desired_points = [self.nuevoPunto() for i in range(0,self.n_drones)]
        elif order == 'abort' :
            self.desired_points = [self.drone_points[i] for i in range(0,self.n_drones)]
        else:
            self.desired_points = [self.nuevoPunto() for i in range(0,self.n_drones)]"""

    def generate_feedback_msg(self):
        for i in range(0, self.n_drones):
            self.distances[i] = self.calc_distance(self.desired_points[i], self.drone_points[i])
        return self.distances    

    def calc_distance(self, p1, p2):
        return sqrt( (p1.x - p2.x)** 2 + (p1.y - p2.y)** 2)

    def points_is_desired(self):
        same_points = True
        i = 0
        while same_points and i < len(self.desired_points):
            same_points = same_points and self.point_equals(self.desired_points[i], self.drone_points[i])
            i = i + 1
        return same_points

    def point_equals(self, p1,p2):
        return round(p1.x) == round(p2.x) and round(p1.y) == round(p2.y) and round(p1.z) == round(p2.z)

    def generate_labeled_paths(self):
        paths = []
        for i in range(0, self.n_drones):
            pi = LabeledPath()
            id = Identifier()
            id.natural = i
            pi.identifier = id
            p1 = self.generateLabeledPoint(0, self.drone_points[i])
            p2 = self.generateLabeledPoint(0, self.desired_points[i])
            pi.points = [p1,p2]
            paths.append(pi)

        return paths

    def generateLabeledPoint(self, label, point):
        p = LabeledPoint()
        l = Label()
        l.natural = label
        p.label = l
        p.point = point
        return p

    def cancel_callback(self, cancel_request):
        """Callback function for the cancel request"""
        self.get_logger().info('Received cancel request')
        self.desired_points = self.drone_points
        plan = Plan()
        plan.paths = self.generate_labeled_paths()
        self.path_publisher.publish(plan)
        return CancelResponse.ACCEPT


def main():
    args = None
    rclpy.init(args=args)

    callback_group = ReentrantCallbackGroup()

    n_drones = 3
    homebase_action_server = HomebaseActionServer(n_drones, callback_group)

    executor = MultiThreadedExecutor()
    executor.add_node(homebase_action_server)

    executor.spin()


if __name__ == '__main__':
    main()