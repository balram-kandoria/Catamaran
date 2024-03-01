import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Launch Plots
os.system('/bin/python3 /home/saturn/Desktop/Dev/catamaran/src/catamaran_visualizer/sensor_data/see_lidar.py')

# Start Nodes

def generate_launch_description():
    pass

    
    return LaunchDescription([
    ])