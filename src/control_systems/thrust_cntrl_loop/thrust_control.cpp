// gz topic -t /wamv/left/thruster/joint/cmd_pos -m gz.msgs.Double -p 'data: -0.15'
// gz topic -t /model/wam-V/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 10.00'

// ros2 topic pub --once wamv/left/thruster/joint/cmd_pos std_msgs/msg/Float64 'data: 1.0'
