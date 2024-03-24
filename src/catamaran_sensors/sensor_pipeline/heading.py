from typing import List
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import math
from geomag import declination
from rosgraph_msgs.msg import Clock

class HeadingPublisher(Node):
    def __init__(self, magnetic_subscription, gps_subscription):
        self.timestamp = -1
        self.yaw = 0
        self.time_int = 1/1000
        super().__init__("heading_publisher")
        self.heading_data_pub = self.create_publisher(Float64, "/wamv/state/heading",10)
        self.magnetic_data_sub = self.create_subscription(MagneticField, magnetic_subscription, self.magnetic_callback, 10)
        self.gps_data_sub = self.create_subscription(NavSatFix, gps_subscription, self.gps_callback, 10)

        self.gps_info = None
        self.magnetic_field_info = None

    def gps_callback(self, data:NavSatFix):
        self.gps_info = data
        self.heading_callback()
    
    def magnetic_callback(self, data:MagneticField):
        self.magnetic_field_info = data
        self.heading_callback()
        

    def heading_callback(self):
        
        if self.magnetic_field_info is not None and self.gps_info is not None:

            azimuth_degrees = Float64()

            # Calculate azimuth (heading)
            decl = declination(math.radians(self.gps_info.latitude), math.radians(self.gps_info.longitude), h= 0)
            azimuth = math.atan2(self.magnetic_field_info.magnetic_field.y, self.magnetic_field_info.magnetic_field.x)  + decl # Result in radians
            azimuth_degrees.data = math.degrees(azimuth)  # Convert radians to degrees

            # Adjust azimuth to range from 0 to 360 degrees
            if azimuth_degrees.data < 0:
                azimuth_degrees.data += 360

            # TODO: Find Bias for specified magnetometer when real-hardware is implemented
            bias = 11.07522539720202
            


            azimuth_degrees.data -= bias

            # TODO: Identify why between true 360 and 250 the azimuth is negative
            if azimuth_degrees.data < 0:
                azimuth_degrees.data += 360

            # self.get_logger().info(f"Publishing: Azimuth {azimuth_degrees}")
            self.heading_data_pub.publish(azimuth_degrees)



def main(args=None):
    rclpy.init(args=args)
    node = HeadingPublisher("/wamv/sensor/magnetometer", "/wamv/sensor/gps")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()