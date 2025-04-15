# Reminder
Make use of TAB when entering command
# Drone launch
```sh
cd ~/colcon_ws
colcon build # if first time / code is modified
. install/setup.bash
ros2 launch drone_bringup drone.launch.py
```
Open a new terminal
```sh
cd ~/ardupilot/Tools/autotest
python3 sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out 127.0.0.1:14551
```
OR 
```sh
cd ~/colcon_ws
./sv.sh
```

Test drone takeoff (In the terminal you execute ./sv.sh)
```sh
mode guided
arm throttle # Wait until drone GPS is available
takeoff 5
```



















# How to drive the drone manually? (CLI) (SKIP)
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