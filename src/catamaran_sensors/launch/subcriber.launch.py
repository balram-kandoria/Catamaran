from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            '/world/waves/model/spherical_buoy/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/lidar/forward/left/points@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            ],
        output='screen',
        remappings=[
        ("/world/waves/model/spherical_buoy/link/imu_link/sensor/imu_sensor/imu", "/imu/spherical_buoy")
        ]
    )

    
    return LaunchDescription([
        bridge
    ])

    