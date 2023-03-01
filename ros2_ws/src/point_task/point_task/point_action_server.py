import rclpy
from rclpy.action import ActionServer, CancelResponse
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

        # Initialize variables
        self.n_drones = n_drones
        self.old_position = [None for i in range(self.n_drones)]
        self.new_position = [None for i in range(self.n_drones)]
        self.images = [None for i in range(self.n_drones)]
        
        # Create a callback group for the action server
        callback_group = ReentrantCallbackGroup()

        # Becomes publisher of the planned_paths topic
        self.publisher_ = self.create_publisher(Plan, '/mutac/planned_paths', 10, 
                        callback_group=callback_group)

        # Subscribes to the performance_metrics topic of each drone
        self.subscription = [self.create_subscription(Metrics, '/mutac/drone'+str(i)+'/performance_metrics', 
                            self.listener_callback, 100, callback_group=callback_group) for i in range(1, self.n_drones+1)]
        
        # Subscribes to the camera topic of each drone
        self.cameras = [self.create_subscription(Image, '/drone_sim_'+str(i)+'/camera', 
                            lambda msg, id=i: self.camera_callback(msg, id), 1, callback_group=callback_group) for i in range(self.n_drones)]
        
        # Creates the action server
        self._action_server = ActionServer(self, PointAction, 'point_action', 
                        self.execute_callback, callback_group=callback_group, cancel_callback=self.cancel_callback)


    def listener_callback(self, msg):
        """Callback function for the performance_metrics topic of each drone"""
        id = msg.identifier.natural-1

        # First time that the drone sends a message
        if self.old_position[id] is None:
            self.old_position[id] = msg.position
        
        # Update the position
        self.new_position[id] = msg.position

    def camera_callback(self, msg, id):
        """Callback function for the camera topic of each drone"""
        self.images[id] = msg

    def execute_callback(self, goal_handle):
        """Callback function for the action server"""
        # Get the desired position from the goal
        msg = goal_handle.request.goal
        self.desired_position = msg

        print('------------------------------------')
        self.get_logger().info('Executing goal '+ str(self.desired_position))
        print('------------------------------------')

        # Publish the path towards the desired position
        msg = self.create_goal_message()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing path:\n'+str(msg))
        print('------------------------------------')
        
        # Wait until the goal is reached
        feedback_msg = PointAction.Feedback()

        while not self.check_goal_reached():
            # Publish the traveled distance as feedback
            feedback_msg.distance = self.create_feedback_message()
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        # Publish the images of the desired position as result
        result = PointAction.Result()
        self.get_logger().info('Goal reached')
        result.result = self.images
        print('------------------------------------')
        return result

    def cancel_callback(self, cancel_request):
        """Callback function for the cancel request"""
        return CancelResponse.ACCEPT
        

    def create_goal_message(self):
        """Creates the message to be published in the planned_paths topic"""
        paths = []

        # Create a path for each drone
        for i in range(self.n_drones):
            identifier = Identifier(natural=i)

            labeled_point_init = self.create_labeled_point(self.old_position[i])
            labeled_point_end = self.create_labeled_point(self.desired_position)
            
            # The path is composed by two points, the actual position and the desired position
            labeled_path = LabeledPath(identifier=identifier, points=[labeled_point_init, labeled_point_end])
            paths.append(labeled_path)

        return Plan(paths=paths)
    
    def create_labeled_point(self, point, label=0):
        """Creates a labeled point"""
        return LabeledPoint(label=Label(natural=label), point=point)
    
    def check_equals(self, pos1, pos2):
        """Checks if two points are equal"""
        return round(pos1.x) == round(pos2.x) and round(pos1.y) == round(pos2.y) and round(pos1.z) == round(pos2.z)

    def create_feedback_message(self):
        """Creates the message to be published as feedback"""
        values = []

        for i in range(self.n_drones):
            # Calculate the distance between the old and the new position
            value = sqrt(
                (self.new_position[i].x - self.old_position[i].x)**2 + 
                (self.new_position[i].y - self.old_position[i].y)**2 +
                (self.new_position[i].z - self.old_position[i].z)**2)
            values.append(value)
        return values

    def check_goal_reached(self):
        """Checks if the goal has been reached"""
        for i in range(self.n_drones):
            if not self.check_equals(self.new_position[i], self.desired_position):
                return False
        return True


def main(args=None):
    rclpy.init(args=None)

    n_drones = 3
    point_action_server = PointActionServer(n_drones)

    executor = MultiThreadedExecutor()
    executor.add_node(point_action_server)

    executor.spin()


if __name__ == '__main__':
    main()