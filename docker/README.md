# Build the container
Run the following commands in the terminal.
1. Build the docker container (Do ONCE in the entire project cycle)
```sh
./build.sh
```
2. In ./run.sh, if you wish to launch with Nvidia GPU, uncomment the GPU part, and comment the non-Nvidia driver part.
OR you can choose not to change the anything, it's still fine.  
```
# # # Specific for non-Nvidia drivers
# docker run -it \
#     --rm \
#     --name drone \
#     -e DISPLAY=$DISPLAY \
#     -e QT_X11_NO_MITSHM=1 \
#     -e XAUTHORITY=$XAUTH \
#     -v "$XAUTH:$XAUTH" \
#     -v "/tmp/.X11-unix:/tmp/.X11-unix" \
#     -v "/etc/localtime:/etc/localtime:ro" \
#     -v "/dev/input:/dev/input" \
#     -v /dev/bus/usb:/dev/bus/usb \
#     --mount type=bind,source=$colcon_ws_path,target=/home/orca4/colcon_ws \
#     --privileged \
#     --security-opt seccomp=unconfined \
#     drone:latest

# Specific for NVIDIA drivers, required for OpenGL >= 3.3
docker run -it \
    --rm \
    --name drone \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    --mount type=bind,source=$colcon_ws_path,target=/home/orca4/colcon_ws \
    --privileged \
    --security-opt seccomp=unconfined \
    --gpus all \
    drone:latest
```

Whenever you want to use the container, run the following command. (Do this if you have closed the container, DON'T do it MULTIPLE times)
```sh
./run.sh
```

3. To have multiple terminals accessing the container (You can do it as many times as you can) 
```sh
docker exec -it drone bash
```
4. Close an active terminal 
```sh
exit
```