import rclpy
from rclpy.node import Node

from sensor_msgs.msg import FluidPressure 

class PressureSubscriber(Node):

    def __init__(self):
        super().__init__('pressure_subscriber')
        # Some QoS policy is required here
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                             history=rclpy.qos.HistoryPolicy.UNKNOWN,
                             durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                             liveliness=rclpy.qos.LivelinessPolicy.AUTOMATIC)
        self.subscription = self.create_subscription(
                msg_type=FluidPressure,
                topic='/pressure',
                callback=self.listener_callback,
                qos_profile=qos_policy)
        self.subscription
        f = open("rosbag_pressure_data.csv", "a")
        f.write("Pressure,Variance;\n")
        f.close()

    def listener_callback(self, pressureData):
        f = open("rosbag_pressure_data.csv", "a")
        f.write(str(pressureData.fluid_pressure) + "," +
                str(pressureData.variance))
        f.close()

def main(args=None):
    rclpy.init(args=args)

    pressure_sub = PressureSubscriber()

    rclpy.spin(pressure_sub)

    pressure_sub.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
