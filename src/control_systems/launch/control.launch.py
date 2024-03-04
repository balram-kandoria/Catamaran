import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Rudder Bridge
    rudder_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/wamv/right/rudder/joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            '/wamv/left/rudder/joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
            ],
        output='screen'
    )

    return LaunchDescription([
        rudder_bridge
    ])