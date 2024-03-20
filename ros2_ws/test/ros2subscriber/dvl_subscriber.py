import rclpy
from rclpy.node import Node

from waterlinked_a50.msg import Transducer, TransducerReport
from waterlinked_a50.msg import TransducerReportStamped

class DVLSubscriber(Node):

    def __init__(self):
        super().__init__('dvl_subscriber')
        self.subscription = self.create_subscription(
                TransducerReportStamped,
                '/velocity_estimate',
                self.listener_callback,
                10)
        self.subscription

        f = open("rosbag_dvl_data.csv", "a")
        f.write("Time,vx,vy,vz,fom;\n")
        f.close()

    def listener_callback(self, dvl_data):
        f = open("rosbag_dvl_data.csv", "a")
        f.write(str(dvl_data.report.time) + "," + str(dvl_data.report.vx) + "," +
                str(dvl_data.report.vy) + "," + str(dvl_data.report.vz) + "," +
                str(dvl_data.report.fom) + ";\n")
        f.close()

def main(args=None):
    rclpy.init(args=args)

    dvl_sub = DVLSubscriber()

    rclpy.spin(dvl_sub)

    dvl_sub.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

