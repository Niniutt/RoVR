import rclpy
from rclpy.node import Node

from ping360_sonar_msgs.msg import SonarEcho 

class SonarSubscriber(Node):

    def __init__(self):
        super().__init__('sonar_subscriber')
        self.subscription = self.create_subscription(
                SonarEcho,
                '/sonar/intensity',
                self.listener_callback,
                10)
        self.subscription
        f = open("rosbag_sonar_data.csv", "a")
        f.write("Intensities;\n")
        f.close()

    def listener_callback(self, sonar):
        f = open("rosbag_sonar_data.csv", "a")
        f.write(str(sonar.intensities) + ";\n")
        f.close()

def main(args=None):
    rclpy.init(args=args)

    sonar_sub = SonarSubscriber()

    rclpy.spin(sonar_sub)

    sonar_sub.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

