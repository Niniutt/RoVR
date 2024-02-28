import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16, String, Int32MultiArray

class SubscriberListener(Node):

    def __init__(self):
        super().__init__('sub')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'info', # Same as in the publisher
            self.listener_callback,
            10
        )
        self.subscription # Prevent warning of undeclared variable

        self.values = [[0], [0]]

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%i"' % msg.data)
        # self.get_logger().info('I heard: "%i"' % msg.data)
        # self.get_logger().info('Active')

        # if msg.data[0] == 0:
        #     self.battery.append(msg.data[1])
        # elif msg.data[0] == 1:
        #     self.depth.append(msg.data[1])

        self.values[msg.data[0]].append(msg.data[1])

        self.get_logger().info('Depth / Battery: "%i / %i" ' % (self.values[0][-1], self.values[1][-1]))

def main(args=None):
    rclpy.init(args=args)

    sub = SubscriberListener()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    