// gz topic -t /wamv/left/thruster/joint/cmd_pos -m gz.msgs.Double -p 'data: -0.15'
// gz topic -t /model/wamv/joint/right_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 10.00'

// ros2 topic pub --once wamv/left/rudder/joint/cmd_pos std_msgs/msg/Float64 'data: 1.0'
// ros2 topic pub --once /wamv/rudder/left/cmd std_msgs/msg/Float64 'data: 1.0'
// ros2 topic pub --once /model/wamv/joint/right_propeller_joint/cmd_thrust std_msgs/msg/Float64 'data: 1.0'

/wamv/thrust/left_propeller/cmd


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

