import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import math

from custom_action_msgs.action import PointAction
from geometry_msgs.msg import Point
from mutac_msgs.msg import LabeledPath, Identifier, LabeledPoint, Label, Plan, Metrics


class PointActionServer(Node):

    def __init__(self, num_drones=1):
        super().__init__('point_action_server')

        self.num_drones = num_drones

        self.old_position = [None for i in range(num_drones)]
        self.new_position = [None for i in range(num_drones)]
        
        callback_group = ReentrantCallbackGroup()

        # Becomes publisher of the planned_paths topic
        self.publisher_ = self.create_publisher(Plan, '/mutac/planned_paths', 10, 
                        callback_group=callback_group)

        # Subscribes to the performance_metrics topic
        self.subscription = self.create_subscription(Metrics, '/mutac/performance_metrics', 
                            self.listener_callback, 10, callback_group=callback_group)
        
        self._action_server = ActionServer(self, PointAction, 'point_action', 
                        self.execute_callback, callback_group=callback_group)  

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        id = msg.identifier.natural-1

        if self.old_position[id] == Point(x=0.0, y=0.0, z=0.0):
            self.old_position[id] = Point(x=msg.position.x, y=msg.position.y, z=msg.position.z)
        self.new_position[id] = Point(x=msg.position.x, y=msg.position.y, z=msg.position.z)

    def create_goal_message(self, msg):
        # Create a labeled path for each drone
        for i in range(self.num_drones):
            identifier = Identifier(natural=i)
            labeled_point_init = self.create_labeled_point(self.old_position[i])
            labeled_point_end = self.create_labeled_point(msg)
            
            labeled_path = LabeledPath(identifier=identifier, points=[labeled_point_init, labeled_point_end])

        return Plan(paths=[labeled_path])
    
    def create_labeled_point(self, point, label=0):
        return LabeledPoint(label=Label(natural=label), point=point)
    
    def check_equals(self, pos1, pos2):
        return pos1.x == pos2.x and pos1.y == pos2.y and pos1.z == pos2.z

    def create_feedback_message(self):
        values = []

        for i in range(self.num_drones):
            value = math.sqrt(
                (self.new_position[i].x - self.old_position[i].x)**2 + 
                (self.new_position[i].y - self.old_position[i].y)**2 +
                (self.new_position[i].z - self.old_position[i].z)**2)
            values.append(value)
        return values

    def check_goal_reached(self):
        for i in range(self.num_drones):
            if not self.check_equals(self.new_position[i], self.desired_position):
                return False
        return True

    def execute_callback(self, goal_handle):
        msg = goal_handle.request.goal
        self.desired_position = Point(x=msg.x, y=msg.y, z=msg.z)

        for i in range(self.num_drones):
            self.old_position[i] = Point(x=0.0, y=0.0, z=0.0)

        self.get_logger().info('Executing goal '+ str(self.desired_position))

        msg = self.create_goal_message(msg)

        self.publisher_.publish(msg)
        
        feedback_msg = PointAction.Feedback()

        while not self.check_goal_reached():
            feedback_msg.distance = self.create_feedback_message()
            #self.get_logger().info('Distance travelled: ' + str(feedback_msg.distance))
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = PointAction.Result()
        result.result = self.desired_position
        return result

def main(args=None):
    rclpy.init(args=None)

    num_drones = 1
    point_action_server = PointActionServer(num_drones)

    executor = MultiThreadedExecutor()
    executor.add_node(point_action_server)

    executor.spin()


if __name__ == '__main__':
    main()