#!/usr/bin/env python3
import sys
import math
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

# Outputs: Active Waypoint, Distance to Active Waypoint (Feedback to thrust controller), Heading to Active Waypoint
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
        self.gps_sub = self.create_subscription(NavSatFix, "/wamv/sensor/gps", self.gps_data_callback, 10)
        self.compass_sub = self.create_subscription(Float64, "/wamv/state/heading", self.heading_data_callback, 10)

        self.gps_data = None
        self.heading_data = None
        
    def absoluteTolatLong(self, x, y, lattitude, longitude, heading, distance = None):

        if distance is None:
            pathLength = (x**2 + y**2)**0.5
        else:
            pathLength = distance

        convertedWaypoint = geopy.distance.distance(meters = pathLength).destination((lattitude, longitude), bearing = heading)

        return (convertedWaypoint.latitude, convertedWaypoint.longitude)

    def computeDestination(self, startLat, startLong, finishLat, finishLong):
        
        distance = geopy.distance.distance((startLat, startLong), (finishLat, finishLong)).m

        return distance 
    
    def computeBearing(self, startLat, startLong, finishLat, finishLong):
        
        startLat = math.radians(startLat)
        startLong = math.radians(startLong)
        finishLat = math.radians(finishLat)
        finishLong = math.radians(finishLong)

        x = math.sin((finishLong - startLong)) * math.cos(finishLat)
        y = (math.cos(startLat) * math.sin(finishLat)) - (math.sin(startLat) * math.cos(finishLat) * math.cos((finishLong - startLong)))

        bearing = (math.atan2(x,y) * (180 / math.pi))
        if bearing < 0:
            bearing += 360
        # angle = math.acos(math.sin(startLat)*math.sin(finishLat) + math.cos(startLat)*math.cos(finishLat)*math.cos((finishLong-startLong)))
        return bearing

    def lawOfCosines(self, a, b, c):
        gamma = math.acos((a**2 + b**2 - c**2)/(2*a*b))
        return gamma
    
    def publish_data(self, Data: NavSatFix):

        thrust_command = Float64()
        # If x position is less than 60 then publish a thrust command to the propellers
        if Data.orientation.x < 60:

            thrust_command.data = 30.0
            self.get_logger().info(f"Thrust Active: {Data.orientation.x}, Current Thrust: {thrust_command}")

            # self.right_prop_pub.publish(thrust_command)
            # self.left_prop_pub.publish(thrust_command)

    def gps_data_callback(self, data: NavSatFix):
        self.gps_data = data
        self.waypoint_callback()
    
    def heading_data_callback(self, data:Float64):
        self.heading_data = data
        self.waypoint_callback()

    def waypoint_callback(self):
        
        if self.gps_data is not None and self.heading_data is not None:
        
            destLat, destLong = self.absoluteTolatLong(x = 20, y = 0, lattitude = self.lattitude, longitude = self.longitude, heading= 90)

            datum_to_current_dist = self.computeDestination(self.gps_data.latitude, self.gps_data.longitude, self.lattitude, self.longitude)
            datum_to_target_dist = self.computeDestination(self.lattitude, self.longitude, destLat,destLong)
            current_to_target_dist = self.computeDestination(self.gps_data.latitude,self.gps_data.longitude, destLat,destLong)

            distance_to_waypoint = self.computeDestination(self.gps_data.latitude,self.gps_data.longitude, destLat,destLong)
            
            bearing = self.computeBearing(destLat,destLong,self.gps_data.latitude,self.gps_data.longitude)
            print(bearing)
            # print(self.heading_data.data)

            if bearing > 180:
                phi = self.heading_data.data
                theta = 180 - ((360 - bearing) + phi)
                rotate = 180 - phi - theta
            else:
                phi = self.heading_data.data
                if phi <= 90:
                    theta = (180 - bearing) + phi
                elif phi <= 180: 
                    theta = (180 - phi) + phi
                else:
                    theta = (180 - bearing) - phi


            print(theta)

            lat, long = self.absoluteTolatLong(x = 20, y = 0, lattitude = self.gps_data.latitude, longitude = self.gps_data.longitude, heading= bearing, distance=current_to_target_dist)
            # print(destLat, destLong)
            # print(lat, long)

            act_waypoint = Vector3()
            act_waypoint.x = self.lattitude
            act_waypoint.y = self.longitude
            act_waypoint.z = 0.0

            # if not (distance_to_waypoint < 10):
            #     self.props.publish_data(30.0)
            # else:
            #     self.props.publish_data(-30.0)
                

            self.active_waypoint_pub.publish(act_waypoint)

            
            # self.publish_data(subscribedData)
    



def main(args=None):
    rclpy.init(args=args)
    node = Waypoints()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()