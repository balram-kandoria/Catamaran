#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rclpy import qos
import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import LaserScan

class lidarVisualizer(Node):
    def __init__(self,nodeName,subTopic):
        self.nodeName = nodeName # String
        self.subscribedTopic = subTopic # String
        super().__init__(self.nodeName)

        self.imu_data_sub = self.create_subscription(LaserScan, self.subscribedTopic, self.plotter_callback, 10)
     



    def plotter_callback(self, data: LaserScan):


        def animate(data):
            
            radians = np.linspace(0,2*np.pi,len(data.ranges))

            fig = plt.figure(num=1, clear=True)

            plt.ion()
            plt.show()

            ax = fig.add_subplot(projection='polar')
            ax.scatter(radians, data.ranges, color="red")

            ax.set_rmax(14)
            ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
            ax.grid(True)

            ax.set_title(self.subscribedTopic, va='bottom')
            
            plt.draw()
            plt.pause(0.001)



        animate(data)
        

    


def main(args=None):
    rclpy.init(args=args)
    node = lidarVisualizer(nodeName="lidar_publisher", subTopic="/lidar/wam_v")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()