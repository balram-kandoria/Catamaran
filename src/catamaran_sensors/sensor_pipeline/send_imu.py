#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Imu
from example_interfaces.msg import Float64
from geometry_msgs.msg import Vector3
import math
from custom_ros_msgs.msg import Dof6
from rosgraph_msgs.msg import Clock

class imuPublisher(Node):
    def __init__(self):
        self.timestamp = -1
        self.yaw = 0
        self.time_int = 1/1000
        super().__init__("imu_publisher")
        self.imu_data_pub = self.create_publisher(Dof6, "/imu/spherical_buoy/dof6",10)
        self.imu_data_sub = self.create_subscription(Imu, "/wamv/sensor/imu", self.imu_callback, 10)
        self.clock_data_sub = self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        
        
    def publish_data(self, Data: Imu):

        newmsg = Dof6()

        acceleration_x = Data.linear_acceleration.x
        acceleration_y = Data.linear_acceleration.y
        acceleration_z = Data.linear_acceleration.z



         # Calculate pitch and roll from accelerometer data
        newmsg.pitch = math.atan2(-acceleration_x, math.sqrt(acceleration_y**2 + acceleration_z**2)) * (180 / math.pi)
        newmsg.roll= math.atan2(acceleration_y, math.sqrt(acceleration_x**2 + acceleration_z**2)) * (180 / math.pi)
        newmsg.yaw = self.yaw + ((Data.angular_velocity.z * self.time_int) * (180 / math.pi))
        newmsg.x = Data.orientation.x
        newmsg.y = Data.orientation.y
        newmsg.z = Data.orientation.z
        
        self.yaw = newmsg.yaw

        self.get_logger().info(f"Publishing: 6dof {newmsg}")
        self.imu_data_pub.publish(newmsg)

    def imu_callback(self, data: Imu):
        subscribedData = data

        self.publish_data(subscribedData)
    
    def clock_callback(self, data: Clock):
        
        currentTime = data._clock.sec + (data._clock.nanosec * 10**(-9))

        if self.timestamp == -1:
            self.timestamp = currentTime
        else:
            self.timestamp_prev = self.timestamp

            self.timestamp = currentTime
        
            deltaT = self.timestamp - self.timestamp_prev
            # print(1/deltaT)


def main(args=None):
    rclpy.init(args=args)
    node = imuPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()