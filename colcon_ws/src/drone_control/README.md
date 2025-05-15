# Control the drone using setpoint
1. Set the drone mode to GUIDED
2. Publish setpoint to /mavros/setpoint_position/local
The frame of local is NED (meter), and relative to the takeoff point.
3. Subscribe /mavros/local_position/odom to get the drone odometry. 

# Demo 
```sh
colcon build # If you haven't compiled yet
. install/setup.bash 
ros2 run drone_control drone_control_demo
```
# Enter values to your program
After launch, to start zig zag search, enter the start and end point. x_s = x_start, y_s = y_end, step size = 1;                     
Execute python3 main.py to start slam.                               
Enter go and drone will move.


