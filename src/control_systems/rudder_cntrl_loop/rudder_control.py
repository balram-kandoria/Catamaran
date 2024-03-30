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
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
import math
sys.path.insert(0,'src/control_systems/')
from thrust_cntrl_loop.thrust_control import ThrustController
import pid

# This class will run everytime the Imu publishes new data
class RudderController(Node):
    def __init__(self):
        num = [1]
        den=[1,2,4]
        self.W = ct.TransferFunction(num,den)

        self.pidcontroller = pid.Controller(dt = 0.001)
        self.cmd,self.integral,self.error,self.feedback = self.pidcontroller.PID(Kp=0.1,Ki=0.001,Kd=0.001,target=0,integral=0,error_prev=0,feedback_prev=0,feedback=0)

        self.timestamp = -1
        self.yaw = 0
        self.time_int = 1/1000
        self.props = ThrustController() # for testing purposes
        super().__init__("rudder_control")
        self.right_rudder_pub = self.create_publisher(Float64, "/wamv/rudder/right/cmd",10)
        self.left_rudder_pub = self.create_publisher(Float64, "/wamv/rudder/left/cmd",10)

        self.imu_sub = self.create_subscription(Imu, "/wamv/sensor/imu", self.imu_callback, 10)
        self.rudder_cmd_sub = self.create_subscription(Float64, "/wamv/state/rotation_command", self.rotation_cmd_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, "/wamv/state/joint_states", self.joint_state_callback, 10)

        self.imu_data = None
        self.rotation_data = None
        self.joint_state_data = None
        
    def imu_callback(self, data:Imu):
        self.imu_data = data
        self.rudder_control()

    def rotation_cmd_callback(self, data: Float64):
        self.rotation_data = data
        self.rudder_control()
    
    def joint_state_callback(self, data: JointState):
        self.joint_state_data = data
        self.rudder_control()

    def rudder_control(self):
        if self.rotation_data is not None and self.imu_data is not None and self.joint_state_data is not None:
            
            self.props.publish_data(5.0)

            # print((self.joint_state_data.name.index('right_engine_joint')))
            right_prop_position = self.joint_state_data._position[0]
            left_prop_position = self.joint_state_data._position[1]

            current_velocity = 1 # m/s

            medium_density = 1000

            area = math.sin(right_prop_position) * 0.25 * 0.5

            mass_flow_rate = medium_density * area * current_velocity

            reaction_y = mass_flow_rate * current_velocity * (math.cos(right_prop_position) - 1)

            sensor_feedback = self.rotation_data.data

            
            self.cmd,self.integral,self.error,self.feedback = self.pidcontroller.PID(Kp=0.1,Ki=0.001,Kd=0.001,target=0,integral=self.integral,error_prev=self.error,feedback_prev=self.feedback,feedback=sensor_feedback)
            print(self.cmd)

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