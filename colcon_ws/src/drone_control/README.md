# Control the drone using setpoint
1. Set the drone mode to GUIDED
2. Publish setpoint to /mavros/setpoint_position/local
The frame of local is NED (meter), and relative to the takeoff point.
3. Subscribe /mavros/local_position/odom to get the drone odometry. 