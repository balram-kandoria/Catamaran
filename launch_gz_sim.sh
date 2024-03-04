# Define Variables
# Start_IMU_PIPELINE_SBuoy="/bin/python3 /home/saturn/Desktop/Dev/catamaran/src/catamaran_sensors/sensor_pipeline/send_imu.py"


# Source catamaran
source /home/saturn/Desktop/Dev/catamaran/install/setup.bash 

# Launch Gazebo
LIBGL_ALWAYS_SOFTWARE=1 gz sim /home/saturn/Desktop/Dev/catamaran/maritime_simulation/worlds/waves.sdf &

# Launch the Gazebo -> ROS bridge for sensor data
ros2 launch catamaran_sensors subcriber.launch.py &

# Launch Visulizer
ros2 launch catamaran_visualizer plotter.launch.py &

# Launch Control Loops
ros2 launch control_systems control.launch.py &

# gnome-terminal -- sh -c "bash -c \"$Start_IMU_PIPELINE_SBuoy; exec bash\"" &

wait