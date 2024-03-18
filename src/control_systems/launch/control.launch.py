import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Launch Plots
os.system('/usr/bin/python3 /home/saturn/Desktop/Dev/catamaran/src/control_systems/waypoints/generate_xml.py')
# os.system('/usr/bin/python3 /home/saturn/Desktop/Dev/catamaran/src/control_systems/thrust_cntrl_loop/thrust_control.py')

def generate_launch_description():
    
    # Rudder Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/wamv/right/rudder/joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            '/wamv/left/rudder/joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wamv/joint/right_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wamv/joint/left_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double'
            ],
        output='screen'
    )

    return LaunchDescription([
        bridge
    ])