#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rclpy import qos
import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64

class Thrust(Node):
    def __init__(self,nodeName,subTopic):
        self.nodeName = nodeName # String
        self.subscribedTopic = subTopic # String
        super().__init__(self.nodeName)

        self.rudder_posn_sub = self.create_subscription(Float64, self.subscribedTopic, self.plotter_callback, 10)
     

    def plotter_callback(self, data: Float64):


        def animate():

            fig = plt.figure(num=2, clear=True)

            plt.ion()
            plt.show()

            def plot_settings(ax):

                ax.set_rmax(1)
                ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
                ax.grid(True)

                # Creates the half polar plot
                ax.set_thetamin(90)
                ax.set_thetamax(-90)

                # ax.set_title(self.subscribedTopic, va='bottom')

            ax = fig.add_subplot(111, projection='polar')
            ax.vlines(0, 0, 1, color="red")

            plot_settings(ax)
            
            plt.draw()
            plt.pause(0.001)



        animate(data)
        





      


def main(args=None):
    rclpy.init(args=args)
    node = Thrust(nodeName="thrust_attributes", subTopic="/wamv/left/rudder/joint/cmd_pos")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main() 