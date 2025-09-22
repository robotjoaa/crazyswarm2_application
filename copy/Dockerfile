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

# install Claude Code dependencies
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        curl \
        wget \
        ca-certificates \
        gnupg2 \
        software-properties-common \
        && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Create startup script to install Claude Code at runtime
RUN mkdir -p /home/user/scripts && \
    cat > /home/user/scripts/install-claude.sh << 'EOF'
#!/bin/bash
if ! command -v claude-code &> /dev/null; then
    echo "Installing Claude Code..."
    # Try official installation method
    curl -fsSL https://claude.ai/install.sh | bash

    # If that fails, try alternative method
    if ! command -v claude-code &> /dev/null; then
        echo "Trying alternative installation..."
        cd /tmp
        wget https://github.com/anthropics/claude-code/releases/latest/download/claude-code-linux-x64.tar.gz 2>/dev/null || \
        wget https://releases.claude.ai/claude-code/claude-code_latest_amd64.deb 2>/dev/null

        if [ -f "claude-code-linux-x64.tar.gz" ]; then
            tar -xzf claude-code-linux-x64.tar.gz
            sudo mv claude-code /usr/local/bin/
            sudo chmod +x /usr/local/bin/claude-code
        elif [ -f "claude-code_latest_amd64.deb" ]; then
            sudo dpkg -i claude-code_latest_amd64.deb || sudo apt-get install -f -y
        fi

        rm -rf /tmp/claude-code* 2>/dev/null
    fi

    # Add to PATH if not already there
    if ! grep -q "claude-code" ~/.bashrc; then
        echo 'export PATH="$HOME/.local/bin:/usr/local/bin:$PATH"' >> ~/.bashrc
        echo 'alias claude="claude-code"' >> ~/.bashrc
    fi

    echo "Claude Code installation completed!"
else
    echo "Claude Code is already installed!"
fi
EOF

# Make the script executable
RUN chmod +x /home/user/scripts/install-claude.sh

# Create desktop shortcut that runs the installer first
RUN mkdir -p ~/Desktop && \
    cat > ~/Desktop/claude-code.desktop << 'EOF'
[Desktop Entry]
Name=Claude Code
Comment=AI-powered code assistant
Exec=bash -c '/home/user/scripts/install-claude.sh && gnome-terminal -- claude-code'
Icon=utilities-terminal
Terminal=false
Type=Application
Categories=Development;IDE;
StartupNotify=true
EOF

RUN chmod +x ~/Desktop/claude-code.desktop

# Also create a menu entry
RUN mkdir -p ~/.local/share/applications && \
    cp ~/Desktop/claude-code.desktop ~/.local/share/applications/

# Add installation script to bashrc so it runs automatically
RUN echo '/home/user/scripts/install-claude.sh' >> ~/.bashrc
