FROM ghcr.io/selkies-project/nvidia-egl-desktop:22.04-20240425164112

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND noninteractive
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        lsb-release \
        less \
        emacs \
        tmux \
        bash-completion \
        command-not-found \
        software-properties-common \
        xdg-user-dirs \
        && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# install ROS2 Humble
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        ros-humble-desktop \
        ros-dev-tools \
        && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# install colcon and rosdep
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# initialize rosdep
RUN sudo rosdep init && \
    rosdep update


# install gazebo garden, git
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        gz-garden \
        git \
        && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*


RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc


# crazyflie-firmware, crazyswarm dependency
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        cmake \
        build-essential \
        libboost-program-options-dev \ 
        libusb-1.0-0-dev \
        ros-humble-motion-capture-tracking \
        ros-humble-tf-transformations \
        && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir Jinja2 rowan nicegui transforms3d==0.4.2


# set working directory 
WORKDIR /home/user/drone 

RUN sudo mkdir -p ./CrazySim/crazyflie-firmware/sitl_make/build

WORKDIR /home/user/drone/ros2_ws

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        python-is-python3 \
        && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# disabled beep sound
RUN echo "set bell-style none" >> ~/.inputrc
RUN echo "export PATH=$HOME/.local/bin:$PATH" >> ~/.bashrc
RUN sudo rm /etc/apt/apt.conf.d/docker-clean
