
import rclpy
import sys
from rclpy.action import ActionServer
from rclpy.node import Node

from homebase_action.action import Homebase
from mutac_msgs.msgs import Plan, LabeledPath, LabeledPoint, Label, Identifier
from geometry_msgs.msgs import Point
from mutac_msgs.msgs import Metrics, Identifier


class HomebaseActionServer(Node):

    def __init__(self, n_drones):
        super().__init__('homebase_action_server')
        self._action_server = ActionServer(
            self,
            Homebase,
            'homebase',
            self.execute_callback)
        

        self.path_publisher = self.create_publisher(LabeledPath, '/mutac/labeled_paths', 100)
        self.pos_listener = self.create_subscription(Metrics, '/mutac/performance_metrics', 100, self.listener_callback)

        self.n_drones = n_drones

        self.accepted_commands = ['go_homebase', 'abort']

        self.drone_points = [Point() for i in range(0,n_drones)]

        self.desired_points = [Point(0,0,0)]

    def listener_callback(self,msg):
        drone_id = msg.identifier.natural
        self.drone_points[drone_id] = msg.position

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        if goal_handle.request.order not in self.accepted_commands:
            goal_handle.canceled()
            return

        plan = Plan()
        plan.paths = self.generate_labeled_paths()
        self.path_publisher.publish(plan)

        # Publicar la posicion de cada dron mientras vuelven
        drones_in_desired = False
        while not drones_in_desired:
            drones_in_desired = self.points_is_desired()
            feedback_msg = Homebdase.Feedback()
            feedback_msg.points = self.drone_points
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()

        result = Homebase.Result()
        result.result = 'Drones en base'


        return result

    def points_is_desired(self):
        same_points = True
        i = 0
        while same_points and i < len(self.desired_points):
            same_points = same_points and point_equals(desired_points[i], drone_points[i])
            i = i + 1
        return same_points

    def point_equals(self, p1,p2):
        return p1.x == p2.x and p1.y == p2.y and p1.z == p2.z

    def generate_labeled_paths(self):
        paths = []
        for i in range(0, self.n_drones):
            pi = LabeledPath()
            pi.identifier = i
            p1 = generateLabeledPoint(0, self.drone_points[i])
            p2 = generateLabeledPoint(0, self.desired_points[i])
            pi.points = [p1,p2]
            paths.append(pi)

    def generateLabeledPoint(self, label, point):
        p = LabeledPoint()
        p.label = label
        p.point = point
        return p



def main(args=None, argv):
    rclpy.init(args=args)

    n_drones = int(argv[0])
    homebase_action_server = HomebaseActionServer(n_drones)

    rclpy.spin(homebase_action_server)


if __name__ == '__main__':
    argv = sys.argv[1:]
    main(args=None, argv)