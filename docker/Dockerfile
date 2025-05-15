# ---------------------------------------------------------------------------
# Build image with ROS 2 Humble Desktop, ArduPilot/ArduSub, ardupilot_gazebo
# and Gazebo Harmonic, but WITHOUT orca4 packages
# ---------------------------------------------------------------------------

    FROM osrf/ros:humble-desktop AS base

    ARG USERNAME=orca4
    ARG USER_UID=1000
    ARG USER_GID=$USER_UID
    ARG DEBIAN_FRONTEND=noninteractive
    
    # ---------------------------------------------------------------------------
    # System update + install handy tools
    # ---------------------------------------------------------------------------
    RUN apt-get update && apt-get upgrade -y
    
    RUN apt-get update \
      && apt-get -y --quiet --no-install-recommends install \
        bash-completion \
        build-essential \
        git \
        glmark2 \
        gnupg \
        iputils-ping \
        lsb-release \
        mlocate \
        software-properties-common \
        sudo \
        wget \
        vim \
        tmux \
      && rm -rf /var/lib/apt/lists/*
    
    # ---------------------------------------------------------------------------
    # Install Gazebo Harmonic for use with ROS 2 Humble
    # ---------------------------------------------------------------------------
    RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    
    RUN apt-get update \
      && apt-get -y --quiet --no-install-recommends install \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
      && rm -rf /var/lib/apt/lists/*
    
    # ---------------------------------------------------------------------------
    # Install NVIDIA software
    # ---------------------------------------------------------------------------
    RUN apt-get update \
      && apt-get -y --quiet --no-install-recommends install \
        libglvnd0 \
        libgl1 \
        libglx0 \
        libegl1 \
        libxext6 \
        libx11-6 \
      && rm -rf /var/lib/apt/lists/*
    
    ENV NVIDIA_VISIBLE_DEVICES=all
    ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
    ENV QT_X11_NO_MITSHM=1
    
    # ---------------------------------------------------------------------------
    # Install additional dependencies (ArduPilot / SITL / GStreamer)
    # ---------------------------------------------------------------------------
    RUN apt-get update \
      && apt-get -y --quiet --no-install-recommends install \
        python3-wxgtk4.0 \
        rapidjson-dev \
        xterm \
        libgz-sim8-dev \
        libopencv-dev \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-libav \
        gstreamer1.0-gl \
      && rm -rf /var/lib/apt/lists/*
    
    # ---------------------------------------------------------------------------
    # Create a non-root user
    # ---------------------------------------------------------------------------
    RUN groupadd --gid $USER_GID $USERNAME \
      && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
      && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
      && chmod 0440 /etc/sudoers.d/$USERNAME \
      && echo "\n# Added by Dockerfile:" >> /home/$USERNAME/.bashrc \
      && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc
    
    # ---------------------------------------------------------------------------
    # Switch to our new user
    # ---------------------------------------------------------------------------
    USER $USERNAME
    ENV USER=$USERNAME
    WORKDIR /home/$USERNAME
    
    # ---------------------------------------------------------------------------
    # Clone ArduSub code
    # ---------------------------------------------------------------------------
    RUN git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules
    
    # ---------------------------------------------------------------------------
    # Install ArduSub prerequisites (adds to ~/.bashrc)
    # ---------------------------------------------------------------------------
    WORKDIR /home/$USERNAME/ardupilot
    ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
    RUN Tools/environment_install/install-prereqs-ubuntu.sh -y
    
    # ---------------------------------------------------------------------------
    # Build ArduSub
    # ---------------------------------------------------------------------------
    RUN modules/waf/waf-light configure --board sitl \    
      && modules/waf/waf-light build --target bin/arducopter \
      && modules/waf/waf-light build --target bin/ardusub
    
    # ---------------------------------------------------------------------------
    # Clone ardupilot_gazebo code
    # ---------------------------------------------------------------------------
    WORKDIR /home/$USERNAME
    RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git
    
    # ---------------------------------------------------------------------------
    # Build for Gazebo Harmonic (define before building ardupilot_gazebo)
    # ---------------------------------------------------------------------------
    ENV GZ_VERSION=harmonic
    
    # ---------------------------------------------------------------------------
    # Build ardupilot_gazebo
    # ---------------------------------------------------------------------------
    RUN /bin/bash -c "\
      cd ardupilot_gazebo \
      && git checkout 4b30a3d8 \
      && mkdir build \
      && cd build \
      && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      && make -j4 \
    "
    
    # ---------------------------------------------------------------------------
    # MAVROS / GeographicLib data install (useful for SITL)
    # ---------------------------------------------------------------------------
    RUN /bin/bash -c "\
      wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
      && chmod +x install_geographiclib_datasets.sh \
      && sudo ./install_geographiclib_datasets.sh \
    "
    # ---------------------------------------------------------------------------
    # (Optional) Enable Matplotlib if you want --console mode in sim_vehicle.py
    # ---------------------------------------------------------------------------
      RUN pip3 install matplotlib
      RUN sudo apt-get install -y ros-humble-mavros
      RUN sudo ./install_geographiclib_datasets.sh
      RUN sudo apt-get install -y ros-humble-tf-transformations

    RUN echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
    RUN echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
    RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
    RUN echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc
    RUN echo 'export GZ_SIM_RESOURCE_PATH=$HOME/colcon_ws/src/drone_description/models:$HOME/colcon_ws/src/drone_description/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
    RUN echo 'source ~/colcon_ws/install/setup.bash' >> ~/.bashrc
    # ---------------------------------------------------------------------------
    # Done
    # ---------------------------------------------------------------------------
    
