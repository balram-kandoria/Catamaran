#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Imu
from example_interfaces.msg import Float64
from geometry_msgs.msg import Vector3
import math

class imuPublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")
        self.imu_data_pub = self.create_publisher(Vector3, "/imu/spherical_buoy/Quaternion",10)
        self.imu_data_sub = self.create_subscription(Imu, "/imu/spherical_buoy", self.imu_callback, 10)
        
        
    def publish_data(self, Data: Imu):

        newmsg = Data.linear_acceleration

        acceleration_x = Data.linear_acceleration.x
        acceleration_y = Data.linear_acceleration.y
        acceleration_z = Data.linear_acceleration.z



         # Calculate pitch and roll from accelerometer data
        pitch_acc = math.atan2(-acceleration_x, math.sqrt(acceleration_y**2 + acceleration_z**2))
        roll_acc = math.atan2(acceleration_y, math.sqrt(acceleration_x**2 + acceleration_z**2))

        self.get_logger().info(f"Publishing: Pitch {pitch_acc} and Roll {roll_acc}")
        self.imu_data_pub.publish(newmsg)

    def imu_callback(self, data: Imu):
        subscribedData = data

        self.publish_data(subscribedData)


def main(args=None):
    rclpy.init(args=args)
    node = imuPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()