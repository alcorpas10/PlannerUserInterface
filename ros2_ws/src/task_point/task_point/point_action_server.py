import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import math
import copy

from accion.action import PointAction
from geometry_msgs.msg import Point, Pose
from mutac_msgs.msg import LabeledPath, Identifier, LabeledPoint, Label, Plan


class PointActionServer(Node):

    def __init__(self):
        super().__init__('point_action_server')

        self.old_position = None
        
        callback_group = ReentrantCallbackGroup()

        # Becomes publisher of the planned_paths topic
        self.publisher_ = self.create_publisher(Plan, '/mutac/planned_paths', 10, 
                        callback_group=callback_group)

        # Subscribes to the drone_pose topic
        self.subscription = self.create_subscription(Pose, '/mutac/drone1/drone_pose', 
                        self.listener_callback, 10, callback_group=callback_group)
        
        self._action_server = ActionServer(self, PointAction, 'point_action', 
                        self.execute_callback, callback_group=callback_group)  

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        print(str(msg.position))
        if self.old_position is None:
            self.old_position = Point(x=msg.position.x, y=msg.position.y, z=msg.position.z)
            self.new_position = Point(x=msg.position.x, y=msg.position.y, z=msg.position.z)
        else:
            self.new_position = Point(x=msg.position.x, y=msg.position.y, z=msg.position.z)

    def create_goal_message(self, msg):
        identifier = Identifier(natural=0)
        labeled_point_init = self.create_labeled_point(self.old_position)
        labeled_point_end = self.create_labeled_point(msg)
        
        labeled_path = LabeledPath(identifier=identifier, points=[labeled_point_init, labeled_point_end])

        return Plan(paths=[labeled_path])

    def create_labeled_point(self, point, label=0):
        return LabeledPoint(label=Label(natural=label), point=point)

    def create_feedback_message(self):
        value = math.sqrt(
            (self.new_position.x - self.old_position.x)**2 + 
            (self.new_position.y - self.old_position.y)**2 + 
            (self.new_position.z - self.old_position.z)**2)

        return round(value)

    def check_equals(self, pos1, pos2):
        #print(str(pos1))
        #print(str(pos2))
        #print(str(round(pos1.x) == round(pos2.x) and round(pos1.y) == round(pos2.y) and round(pos1.z) == round(pos2.z)))

        return (round(pos1.x) == round(pos2.x) and
            round(pos1.y) == round(pos2.y) and
            round(pos1.z) == round(pos2.z))

    def execute_callback(self, goal_handle):
        msg = goal_handle.request.goal
        self.desired_position = Point(x=msg.x, y=msg.y, z=msg.z)
        self.get_logger().info('Executing goal '+ str(self.desired_position))

        msg = self.create_goal_message(msg)

        self.publisher_.publish(msg)
        
        feedback_msg = PointAction.Feedback()

        while not self.check_equals(self.new_position, self.desired_position):
            feedback_msg.distance = self.create_feedback_message()
            #self.get_logger().info('Distance travelled: ' + str(feedback_msg.distance))
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = PointAction.Result()
        result.result = self.new_position
        return result

def main(args=None):
    rclpy.init(args=None)

    point_action_server = PointActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(point_action_server)

    executor.spin()


if __name__ == '__main__':
    main()