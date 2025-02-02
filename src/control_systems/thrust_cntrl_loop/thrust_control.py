#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock

# This class will run everytime the Imu publishes new data
class ThrustController(Node):
    def __init__(self):
        self.timestamp = -1
        self.yaw = 0
        self.time_int = 1/1000
        super().__init__("thrust_control")
        self.right_prop_pub = self.create_publisher(Float64, "/wamv/thrust/right_propeller/cmd",10)
        self.left_prop_pub = self.create_publisher(Float64, "/wamv/thrust/left_propeller/cmd",10)
        self.prop_sub = self.create_subscription(Imu, "/wamv/sensor/imu", self.imu_callback, 10)
        
        
    def publish_data(self, thrust):

        thrust_command = Float64()

        # If x position is less than 60 then publish a thrust command to the propellers

        thrust_command.data = float(thrust)
        # self.get_logger().info(f"Thrust Active: Current Thrust: {thrust_command}")

        self.right_prop_pub.publish(thrust_command)
        self.left_prop_pub.publish(thrust_command)

    def imu_callback(self, data: Imu):
        subscribedData = data

        self.publish_data(150)
    



def main(args=None):
    rclpy.init(args=args)
    node = ThrustController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()