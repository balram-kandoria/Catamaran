import os
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
            '/lidar/bow/port@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/world/waves/model/hull/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'

            ],
        output='screen',
        remappings=[
        ("/world/waves/model/spherical_buoy/link/imu_link/sensor/imu_sensor/imu", "/imu/spherical_buoy"),
        ("/world/waves/model/hull/joint_state", "/joint_states")
        ]
    )

    sdf_file = os.path.join('maritime_simulation', 'models', 'hull', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[sdf_file],
        output=['screen']
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    
    return LaunchDescription([
        bridge,
        robot_state_publisher
    ])

    