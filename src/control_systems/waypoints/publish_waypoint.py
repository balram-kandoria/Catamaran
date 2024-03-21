#!/usr/bin/env python3
import sys
from typing import List
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
import geopy.distance
# Custom Imports
from read_json import loadJson
sys.path.insert(0,'src/control_systems/')
from thrust_cntrl_loop.thrust_control import ThrustController

# This class will run everytime the Imu publishes new data

# Outputs: Active Waypoint, Distance to Active Waypoint, Heading to Active Waypoint
# Inputs: Current GPS position
# Parameters: Threshold to reach Waypoint
# Overview: Provide Heading and Distance commands to controllers when Waypoint Following is engaged
# - Determine when a Waypoint has been reached and change the active waypoint
class Waypoints(Node):
    # 1. Determine active waypoint 
    #   a. Need way to determine when waypoint is reached and change active waypoint (if within 1 m)
    # 2. Determine Heading / Bearing needed for waypoint https://stackoverflow.com/questions/3932502/calculate-angle-between-two-latitude-longitude-points
    # 3. Determine distance to waypoint (if waypoint is specified as lat/long) distance.distance(wellington, salamanca).m [DONE]
    # 4. Compute waypoint lat/long (if waypoint is specified as x,y) geopy.distance.distance(miles=10).destination((34, 148), bearing=90) [DONE]

    # +x 90, +y 0, -y 180, -x 270
    def __init__(self):

        # World Lattitude and Longitude are used as a datum reference for waypoints which are absolute values
        self.longitude = -122.218082 # referenced /worlds/waves.sdf
        self.lattitude = 47.699345 # referenced /worlds/waves.sdf

        self.waypoints = loadJson()
        self.props = ThrustController() # for testing purposes

        super().__init__("Waypoints")
        self.active_waypoint_pub = self.create_publisher(Vector3, "/world/navigation/active_waypoint",10)
        self.gps_sub = self.create_subscription(NavSatFix, "/wamv/sensor/gps", self.waypoint_callback, 10)
        
    def absoluteTolatLong(self, x, y, lattitude, longitude, heading):

        pathLength = (x**2 + y**2)**0.5
        convertedWaypoint = geopy.distance.distance(meters = pathLength).destination((lattitude, longitude), bearing = heading)

        return (convertedWaypoint.latitude, convertedWaypoint.longitude)

    def computeDestination(self, startLat, startLong, finishLat, finishLong):
        
        distance = geopy.distance.distance((startLat, startLong), (finishLat, finishLong)).m

        return distance 
    
    def publish_data(self, Data: NavSatFix):

        thrust_command = Float64()
        # If x position is less than 60 then publish a thrust command to the propellers
        if Data.orientation.x < 60:

            thrust_command.data = 30.0
            self.get_logger().info(f"Thrust Active: {Data.orientation.x}, Current Thrust: {thrust_command}")

            # self.right_prop_pub.publish(thrust_command)
            # self.left_prop_pub.publish(thrust_command)

    def waypoint_callback(self, data: NavSatFix):
        
        destLat, destLong = self.absoluteTolatLong(x = 20, y = 0, lattitude = self.lattitude, longitude = self.longitude, heading= 90)

        act_waypoint = Vector3()

        distance_to_waypoint = self.computeDestination(data.latitude,data.longitude, destLat,destLong)
        
        act_waypoint.x = self.lattitude
        act_waypoint.y = self.longitude
        act_waypoint.z = 0.0

        if not (distance_to_waypoint < 10):
            self.props.publish_data(30.0)
        else:
            self.props.publish_data(-30.0)
            

        self.active_waypoint_pub.publish(act_waypoint)

        
        # self.publish_data(subscribedData)
    



def main(args=None):
    rclpy.init(args=args)
    node = Waypoints()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()