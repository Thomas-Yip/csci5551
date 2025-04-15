#!/usr/bin/env bash

# Define the Xauthority file location
XAUTH=/tmp/.docker.xauth
export DISPLAY=:0
# Create or update the .docker.xauth file
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Run the Docker container with GUI support (without NVIDIA settings)
current_dir=$(pwd)
orca4_path=$(dirname "$current_dir")
colcon_ws="colcon_ws"
colcon_ws_path="${orca4_path}/${colcon_ws}"
echo "Colcon workspace path: $colcon_ws_path"

# # Specific for non-Nvidia drivers
docker run -it \
    --rm \
    --name drone \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    -v /dev/bus/usb:/dev/bus/usb \
    --mount type=bind,source=$colcon_ws_path,target=/home/orca4/colcon_ws \
    --privileged \
    --security-opt seccomp=unconfined \
    drone:latest

# # Specific for NVIDIA drivers, required for OpenGL >= 3.3
# docker run -it \
#     --rm \
#     --name drone \
#     -e DISPLAY \
#     -e QT_X11_NO_MITSHM=1 \
#     -e XAUTHORITY=$XAUTH \
#     -e NVIDIA_VISIBLE_DEVICES=all \
#     -e NVIDIA_DRIVER_CAPABILITIES=all \
#     -v "$XAUTH:$XAUTH" \
#     -v "/tmp/.X11-unix:/tmp/.X11-unix" \
#     -v "/etc/localtime:/etc/localtime:ro" \
#     -v "/dev/input:/dev/input" \
#     --mount type=bind,source=$colcon_ws_path,target=/home/orca4/colcon_ws \
#     --privileged \
#     --security-opt seccomp=unconfined \
#     --gpus all \
#     drone:latest
