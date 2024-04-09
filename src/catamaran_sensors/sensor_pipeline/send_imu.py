#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Imu
from example_interfaces.msg import Float64
from geometry_msgs.msg import Vector3
import math
from rosgraph_msgs.msg import Clock

class imuPublisher(Node):
    def __init__(self):
        self.timestamp = -1
        self.yaw = 0
        self.time_int = 0.001
        super().__init__("imu_publisher")
        self.velocity_pub = self.create_publisher(Vector3, "/wamv/state/linear_velocity",10)
        self.imu_data_sub = self.create_subscription(Imu, "/wamv/sensor/imu", self.imu_callback, 10)

        self.x = 0
        self.y = 0
        self.z = 0
        self.delt = 0
        

    def imu_callback(self, data: Imu):
        
        linearVelocity = Vector3()
        if self.x == 0 and self.y == 0 and self.z == 0:
            self.x = data.orientation.x
            self.y = data.orientation.y
            self.z = data.orientation.z
        else:
            velocityX = (data.orientation.x - self.x) / self.time_int
            velocityY = (data.orientation.y - self.y) / self.time_int
            velocityZ = (data.orientation.z - self.z) / self.time_int

            self.x = data.orientation.x
            self.y = data.orientation.y
            self.z = data.orientation.z

            linearVelocity.x = velocityX
            linearVelocity.y = velocityY
            linearVelocity.z = velocityZ

            self.velocity_pub.publish(linearVelocity)
    


def main(args=None):
    rclpy.init(args=args)
    node = imuPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()