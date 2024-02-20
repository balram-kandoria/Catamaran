#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Imu
from example_interfaces.msg import Float64

class imuPublisher(Node):
    def __init__(self):
        super().__init__("imu_publisher")
        self.imu_data_sub = self.create_subscription(Imu, "/imu/spherical_buoy", self.imu_callback, 10)
        self.imu_data_pub = self.create_publisher(Imu, "/imu/spherical_buoy/raw",10)
        self.timer = self.create_timer(1.0, self.imu_data_sub)
        
    def publish_data(self, newData: Imu):
        newmsg = newData
        self.get_logger().info(f"Publishing: {newmsg}")
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