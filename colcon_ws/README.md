# Drone launch
## Launch the simulation environment
```sh
cd ~/colcon_ws
colcon build # if first time / code is modified
. install/setup.bash
ros2 launch drone_bringup drone.launch.py
```
## Connect the UAV flight controller to ROS
Open a new terminal
```sh
cd ~/colcon_ws
./sv.sh
```
OR 
```sh
cd ~/ardupilot/Tools/autotest
python3 sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out 127.0.0.1:14551
```
## UAV takeoff
Test drone takeoff (In the terminal you execute ./sv.sh)
```sh
mode guided
arm throttle # Wait until drone GPS is available
takeoff 10 # Set UAV takeoff altitude
```
## Run EKF SLAM
```sh
cd ~/colcon_ws/src/ekf_slam/ekf_slam/src
python3 main.py
```
## Launch zig zag search
```sh
ros2 run drone_control drone_control_demo 
```
Enter start coordinate (x,y), dest coordinate (x,y), drone altitude, step size in x-direction, step size in y-direction.      
After entering all information, enter "go" and drone will start searching.  



















# How to drive the drone manually? (CLI) (SKIP EVERYTHING BELOW)
## Run Gazebo
```sh
gz sim -v4 -r iris_runway.sdf
```

## Run Mavproxy
```sh
cd ~/ardupilot/Tools/autotest
python3 sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out 127.0.0.1:14551
```

### You can control the drone manually or by ROS

#### Manual

## Arm and takeoff
```sh
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```
## Land
```sh
land
```

#### ROS
In the mavproxy terminal, enter
```sh
output add 127.0.0.1:14551
```

Open another termainl, launch mavros
```sh
cd ~/colcon_ws
. install/setup.bash
ros2 launch drone_bringup drone.launch.py
```

Optional
To monitor the state of the drone, run
```sh
ros2 topic echo /mavros/state
```

# mavproxy.py
export PATH=$PATH:$HOME/.local/bin

# Build the workspace
1. colcon build
# Source the workspace
2. . install/setup.bash
3. Launch drone
```sh
ros2 launch drone_bringup drone.launch.py
```
4. 
```sh
cd ~/colcon_ws
./sv.sh
```
