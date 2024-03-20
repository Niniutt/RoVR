import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

class IMUSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
                Imu,
                'imu/data',
                self.listener_callback,
                10)
        self.subscription
        f = open("rosbag_data.csv", "a")
        f.write("Orientation:x,Orientation:y,Orientation:z,Orientation:w,AngVel:x,AngVel:y,AngVel:z,Acc:x,Acc:y,Acc:z;\n")
        f.close()

    def listener_callback(self, imu):
        f = open("rosbag_data.csv", "a")
        f.write(str(imu.orientation.x) + "," + str(imu.orientation.y) + "," +
                str(imu.orientation.z) + "," + str(imu.orientation.w) + "," +
                str(imu.angular_velocity.x) + "," + str(imu.angular_velocity.y)
                + "," + str(imu.angular_velocity.z) + "," +
                str(imu.linear_acceleration.x) + "," +
                str(imu.linear_acceleration.y) + "," +
                str(imu.linear_acceleration.z) + ";\n")
        f.close()

def main(args=None):
    rclpy.init(args=args)

    imu_sub = IMUSubscriber()

    rclpy.spin(imu_sub)

    imu_sub.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
