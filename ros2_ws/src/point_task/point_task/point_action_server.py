import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from math import sqrt

from custom_action_msgs.action import PointAction
from geometry_msgs.msg import Point
from mutac_msgs.msg import LabeledPath, Identifier, LabeledPoint, Label, Plan, Metrics
from sensor_msgs.msg import Image


class PointActionServer(Node):

    def __init__(self, n_drones=1):
        super().__init__('point_action_server')

        self.n_drones = n_drones

        self.old_position = [None for i in range(self.n_drones)]
        self.new_position = [None for i in range(self.n_drones)]
        self.images = [None for i in range(self.n_drones)]
        
        callback_group = ReentrantCallbackGroup()

        # Becomes publisher of the planned_paths topic
        self.publisher_ = self.create_publisher(Plan, '/mutac/planned_paths', 10, 
                        callback_group=callback_group)

        # Subscribes to the performance_metrics topic

        self.subscription = [self.create_subscription(Metrics, '/mutac/drone'+str(i)+'/performance_metrics', 
                            self.listener_callback, 100, callback_group=callback_group) for i in range(1, self.n_drones+1)]
        
        self._action_server = ActionServer(self, PointAction, 'point_action', 
                        self.execute_callback, callback_group=callback_group)

        self.cameras = [self.create_subscription(Image, '/drone_sim_'+str(i)+'/camera', 
                            lambda msg, id=i: self.camera_callback(msg, id), 1, callback_group=callback_group) for i in range(self.n_drones)]
        
    def camera_callback(self, msg, id):
        self.images[id] = msg

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        id = msg.identifier.natural-1

        if self.old_position[id] is None:
            #self.old_position[id] = Point(x=msg.position.x, y=msg.position.y, z=msg.position.z)
            self.old_position[id] = msg.position
        #self.new_position[id] = Point(x=msg.position.x, y=msg.position.y, z=msg.position.z)
        self.new_position[id] = msg.position

    def create_goal_message(self):
        # Create a labeled path for each drone
        paths = []

        for i in range(self.n_drones):
            identifier = Identifier(natural=i)
            labeled_point_init = self.create_labeled_point(self.old_position[i])
            labeled_point_end = self.create_labeled_point(self.desired_position)
            
            labeled_path = LabeledPath(identifier=identifier, points=[labeled_point_init, labeled_point_end])
            paths.append(labeled_path)

        return Plan(paths=paths)
    
    def create_labeled_point(self, point, label=0):
        return LabeledPoint(label=Label(natural=label), point=point)
    
    def check_equals(self, pos1, pos2):
        return round(pos1.x) == round(pos2.x) and round(pos1.y) == round(pos2.y) and round(pos1.z) == round(pos2.z)

    def create_feedback_message(self):
        values = []

        for i in range(self.n_drones):
            value = sqrt(
                (self.new_position[i].x - self.old_position[i].x)**2 + 
                (self.new_position[i].y - self.old_position[i].y)**2 +
                (self.new_position[i].z - self.old_position[i].z)**2)
            values.append(value)
        return values

    def check_goal_reached(self):
        for i in range(self.n_drones):
            if not self.check_equals(self.new_position[i], self.desired_position):
                return False
        return True

    def execute_callback(self, goal_handle):        
        msg = goal_handle.request.goal
        self.desired_position = msg

        print('------------------------------------')
        self.get_logger().info('Executing goal '+ str(self.desired_position))
        print('------------------------------------')

        msg = self.create_goal_message()

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing path:\n'+str(msg))
        print('------------------------------------')
        
        feedback_msg = PointAction.Feedback()

        while not self.check_goal_reached():
            feedback_msg.distance = self.create_feedback_message()
            #self.get_logger().info('Distance travelled: ' + str(feedback_msg.distance))
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = PointAction.Result()
        self.get_logger().info('Goal reached')
        result.result = self.images
        print('------------------------------------')
        return result


def main(args=None):
    rclpy.init(args=None)

    n_drones = 3
    point_action_server = PointActionServer(n_drones)

    executor = MultiThreadedExecutor()
    executor.add_node(point_action_server)

    executor.spin()


if __name__ == '__main__':
    main()