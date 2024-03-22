from typing import List
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import math
from rosgraph_msgs.msg import Clock

class HeadingPublisher(Node):
    def __init__(self, subscription):
        self.timestamp = -1
        self.yaw = 0
        self.time_int = 1/1000
        super().__init__("heading_publisher")
        self.heading_data_pub = self.create_publisher(Float64, "/wamv/state/heading",10)
        self.heading_data_sub = self.create_subscription(MagneticField, subscription, self.heading_callback, 10)
        
        
    def heading_callback(self, data: MagneticField):
        
        azimuth_degrees = Float64()
        # Calculate azimuth (heading)
        azimuth = math.atan2(data.magnetic_field.y, data.magnetic_field.x)  # Result in radians
        azimuth_degrees.data = math.degrees(azimuth)  # Convert radians to degrees

        # Adjust azimuth to range from 0 to 360 degrees
        if azimuth_degrees.data < 0:
            azimuth_degrees.data += 360

        # self.get_logger().info(f"Publishing: Azimuth {azimuth_degrees}")
        self.heading_data_pub.publish(azimuth_degrees)



def main(args=None):
    rclpy.init(args=args)
    node = HeadingPublisher("/wamv/sensor/magnetometer")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()