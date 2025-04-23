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

```cpp
  float setPoints[5][3] = { 
    {0, 0, 10},
    {10, 0, 10},
    {10, 10, 10},
    {0, 10, 10},
    {0, 0, 10}
  };
  int current_setpoint_ind = 0; // Index to track the current setpoint.
  int final_setpoint_ind = 4;
```
You can add more setpoints to create a new flight path.
The setpoint index is updated when the odometry shows the drone flies to the setpoint. 

# setpoint generation function
If I want to do a grid search, let's say the search area is (10m x 10m), I should describe the corner coordinates of the search area relative to the starting point, and the function will generate waypoints for the drone to follow. 
Examples (altitude omitted):
1. (0, 0), (10, 0), (10, 10), (10, 0)
2. (5, -5), (5, 5), (-5, 5), (-5, -5)