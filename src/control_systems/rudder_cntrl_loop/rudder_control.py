#!/usr/bin/env python3
import sys
from typing import List
import numpy as np
import control as ct
import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
import math
from matplotlib import pyplot as plt
sys.path.insert(0,'src/control_systems/')
from waypoints.publish_waypoint import Waypoints
from thrust_cntrl_loop.thrust_control import ThrustController
import pid

# This class will run everytime the Imu publishes new data
class RudderController(Node):
    def __init__(self):
        num = [1]
        den=[1,2,4]
        self.W = ct.TransferFunction(num,den)

        self.time = 0
        self.time_vec = []
        self.output = []
        
        self.pidcontroller = pid.Controller(dt = 0.001)
        self.cmd,self.integral,self.error,self.feedback = self.pidcontroller.PID(Kp=0.1,Ki=0.001,Kd=0.01,target=0,integral=0,error_prev=0,feedback_prev=0,feedback=0)

        self.timestamp = -1
        self.yaw = 0
        self.time_int = 1/1000
        self.props = ThrustController() # for testing purposes
        self.waypointObj = Waypoints()
        super().__init__("rudder_control")
        self.right_rudder_pub = self.create_publisher(Float64, "/wamv/rudder/right/cmd",10)
        self.left_rudder_pub = self.create_publisher(Float64, "/wamv/rudder/left/cmd",10)

        self.imu_sub = self.create_subscription(Imu, "/wamv/sensor/imu", self.imu_callback, 10)
        self.rudder_cmd_sub = self.create_subscription(Float64, "/wamv/state/rotation_command", self.rotation_cmd_callback, 10)
        self.joint_state_sub = self.create_subscription(Vector3, "/world/navigation/active_waypoint", self.active_waypoint_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, "/wamv/sensor/gps", self.gps_data_callback, 10)
        self.compass_sub = self.create_subscription(Float64, "/wamv/state/heading", self.heading_data_callback, 10)

        self.imu_data = None
        self.rotation_data = None
        self.active_waypoint = None
        self.gps_data = None
        self.heading_data = None
        
    def imu_callback(self, data:Imu):
        self.imu_data = data
        self.rudder_control()

    def rotation_cmd_callback(self, data: Float64):
        self.rotation_data = data
        self.rudder_control()
    
    def active_waypoint_callback(self, data: Vector3):
        self.active_waypoint = data
        self.rudder_control()

    def gps_data_callback(self, data: NavSatFix):
        self.gps_data = data
        self.rudder_control()
    
    def heading_data_callback(self, data:Float64):
        self.heading_data = data
        self.rudder_control()

    def rudder_control(self):
        if self.rotation_data is not None and self.imu_data is not None and self.heading_data is not None and self.gps_data is not None and self.active_waypoint is not None:
            
            # self.props.publish_data(10.0)

            # print((self.joint_state_data.name.index('right_engine_joint')))
            # right_prop_position = self.joint_state_data._position[0]
            # left_prop_position = self.joint_state_data._position[1]

            # current_velocity = 1 # m/s

            # medium_density = 1000

            # area = math.sin(right_prop_position) * 0.25 * 0.5

            # mass_flow_rate = medium_density * area * current_velocity

            # reaction_y = mass_flow_rate * current_velocity * (math.cos(right_prop_position) - 1)

            sensor_feedback = self.rotation_data.data
            distanceToTarget = self.waypointObj.computeDestination(self.active_waypoint.x,self.active_waypoint.y, self.gps_data.latitude,self.gps_data.longitude)
            print(f"Distance to Target = {distanceToTarget}")
            desiredState = np.array([0,0])
            currentState = np.array([distanceToTarget, sensor_feedback])
            heading = self.heading_data.data
            linearVelocity = 10
            angularVelocity = self.imu_data.angular_velocity.z
            
            # self.cmd,self.integral,self.error,self.feedback = self.pidcontroller.PID(Kp=0.8,Ki=0.05,Kd=0.01,target=0,integral=self.integral,error_prev=self.error,feedback_prev=self.feedback,feedback=sensor_feedback)
            self.cmd, self.thrustCmd = self.pidcontroller.LQR(desiredState, currentState, heading, linearVelocity, angularVelocity)
            print(f"Thrust = {(self.thrustCmd)}")
            rudderCMD = Float64()
            if abs(self.cmd) > 1.2:
                self.cmd = 1.2 * (self.cmd / abs(self.cmd))
                self.props.publish_data((self.thrustCmd))
            
            rudderCMD.data = -self.cmd

            self.right_rudder_pub.publish(rudderCMD)
            self.left_rudder_pub.publish(rudderCMD)
            print(f"Rudder Command = {rudderCMD.data}")

            self.imu_data = None
            self.rotation_data = None

            self.time += 1/100
            self.time_vec.append(self.time)
            self.output.append(self.cmd)
            

            fig = plt.figure(num=2, clear=True)

            plt.ion()
            plt.show()

            ax = fig.add_subplot()
            ax.scatter(self.time_vec, self.output, color="red")

            ax.grid(True)

            ax.set_title("Tuning", va='bottom')
            
            plt.draw()
            plt.pause(0.001)




            # print(self.joint_state_data.name[np.where(self.joint_state_data.name == 'left_engine_joint')])

    def publish_data(self, thrust):

        thrust_command = Float64()

        # If x position is less than 60 then publish a thrust command to the propellers

        thrust_command.data = float(thrust)
        self.get_logger().info(f"Thrust Active: Current Thrust: {thrust_command}")

        self.right_prop_pub.publish(thrust_command)
        self.left_prop_pub.publish(thrust_command)

    

def main(args=None):
    rclpy.init(args=args)
    node = RudderController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()