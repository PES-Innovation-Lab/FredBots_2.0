FROM ubuntu:22.04

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV TZ=Etc/UTC

RUN apt install software-properties-common && add-apt-repository universe

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common \
    sudo git wget openssh-server \
    python3 python3-pip \
    locales  && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-nav2 ros-humble-turtlebot3-navigation \
    build-essential cmake python3-rosdep python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir -U colcon-common-extensions

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /ros2_ws

RUN mkdir -p /ros2_ws/src && \
    git clone https://github.com/PES-Innovation-Lab/FredBots_2.0.git /ros2_ws/src/FredBots_2.0

RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

RUN mkdir /var/run/sshd && \
    echo 'root:password' | chpasswd && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    ssh-keygen -A

EXPOSE 22

CMD service ssh start && bash
