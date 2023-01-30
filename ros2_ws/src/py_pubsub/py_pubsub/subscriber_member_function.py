import rclpy
from rclpy.node import Node

import numpy as np

import matplotlib.pyplot as plt

from sensor_msgs.msg import Image

class MinimalSubscriber(Node):


    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription1 = self.create_subscription(
            Image,                                              # CHANGE
            '/rgb1',
            self.battery_callback,
            10)
        self.subscription1
        self.impreso = False

    def battery_callback(self, msg):
        if not self.impreso :
            Im = np.array(msg.data)
            self.get_logger().info(str(Im[0:9]))
            Im = Im.reshape(-1, 3)
            self.get_logger().info(str(Im[0:3,:]))
            colors = [Im[:,i] for i in range(0,3)]
            self.get_logger().info(str(colors[0].shape))
            colors_reshape = [i.reshape(360,640) for i in colors]
            r, g, b = colors_reshape[0], colors_reshape[1], colors_reshape[2]
            colors_reshape = [b, g, r]
            Im2 = np.stack(colors_reshape, axis=-1)
            self.get_logger().info(" ---- " + str(Im2.shape) + " --- ")
            plt.imshow(Im2)
            plt.show()
            self.get_logger().info(str(Im.shape))
            self.get_logger().info(str(msg.step))
            self.get_logger().info(str(msg.width))
            self.get_logger().info(str(msg.height))
            self.impreso = True
        #self.get_logger().info('I heard: "%d"' % msg.percentaje) # CHANGE



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    while(rclpy.ok()):
        rclpy.spin_once(minimal_subscriber)


    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()