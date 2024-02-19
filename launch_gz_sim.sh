# Source catamaran
source /home/saturn/Desktop/Dev/catamaran/install/setup.bash 

# Launch Gazebo
LIBGL_ALWAYS_SOFTWARE=1 gz sim waves.sdf &

# Launch the Gazebo -> ROS bridge for sensor data
ros2 launch catamaran_sensors subcriber.launch.py &

wait