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