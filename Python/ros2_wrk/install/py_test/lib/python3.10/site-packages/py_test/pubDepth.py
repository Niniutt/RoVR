import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int16, Int32MultiArray

class PublisherDepth(Node):
    def __init__(self):
        super().__init__('pubDepth')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'info', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = [0, self.i]
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%i / %i"' % (msg.data[0], msg.data[1]))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    pubDepth = PublisherDepth()

    rclpy.spin(pubDepth)

    pubDepth.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()