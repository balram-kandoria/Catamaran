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
class Waypoints(Node):
    # 1. Determine active waypoint 
    #   a. Need way to determine when waypoint is reached and change active waypoint (if within 1 m)
    # 2. Determine Heading / Bearing needed for waypoint https://stackoverflow.com/questions/3932502/calculate-angle-between-two-latitude-longitude-points
    # 3. Determine distance to waypoint (if waypoint is specified as lat/long) distance.distance(wellington, salamanca).m
    # 4. Compute waypoint lat/long (if waypoint is specified as x,y) geopy.distance.distance(miles=10).destination((34, 148), bearing=90)

    def __init__(self):

        # World Lattitude and Longitude are used as a datum reference for waypoints which are absolute values
        self.longitude = -122.218082 # referenced /worlds/waves.sdf
        self.lattitude = 47.699345 # referenced /worlds/waves.sdf

        super().__init__("Waypoints")
        self.left_prop_pub = self.create_publisher(Float64, "/model/wamv/joint/left_propeller_joint/cmd_thrust",10)
        self.prop_sub = self.create_subscription(Imu, "/wamv/sensor/gps", self.imu_callback, 10)
        
        
    def publish_data(self, Data: Imu):

        thrust_command = Float64()

        # If x position is less than 60 then publish a thrust command to the propellers
        if Data.orientation.x < 60:

            thrust_command.data = 30.0
            self.get_logger().info(f"Thrust Active: {Data.orientation.x}, Current Thrust: {thrust_command}")

            self.right_prop_pub.publish(thrust_command)
            self.left_prop_pub.publish(thrust_command)

    def imu_callback(self, data: Imu):
        subscribedData = data

        self.publish_data(subscribedData)
    



def main(args=None):
    rclpy.init(args=args)
    node = Waypoints()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()