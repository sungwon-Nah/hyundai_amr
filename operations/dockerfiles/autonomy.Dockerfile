FROM osrf/ros:humble-desktop-full

# ARG WORKSPACE=workspace
# WORKDIR /root/$WORKSPACE

# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
# ENV QT_X11_NO_MITSHM=1
# ENV EDITOR=nano
# ENV XDG_RUNTIME_DIR=/tmp

## Install from packages list
COPY ${HYUNDAI_AMR_ROOT}/operations/dockerfiles/apt-packages /tmp/
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y gettext-base && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages \
    && apt-get clean

# RUN apt-get update && apt-get install -y \
#     cmake \
#     curl \
#     gazebo \
#     libglu1-mesa-dev \
#     nano \
#     python3-pip \
#     python3-pydantic \
#     ros-humble-gazebo-ros \
#     ros-humble-gazebo-ros-pkgs \
#     ros-humble-joint-state-publisher \
#     ros-humble-robot-localization \
#     ros-humble-plotjuggler-ros \
#     ros-humble-robot-state-publisher \
#     ros-humble-ros2bag \
#     ros-humble-rosbag2-storage-default-plugins \
#     ros-humble-rqt-tf-tree \
#     ros-humble-rmw-fastrtps-cpp \
#     ros-humble-rmw-cyclonedds-cpp \
#     ros-humble-slam-toolbox \
#     ros-humble-turtlebot3 \
#     ros-humble-turtlebot3-msgs \
#     ros-humble-twist-mux \
#     ros-humble-usb-cam \
#     ros-humble-xacro \
#     ruby-dev \
#     rviz \
#     tmux \
#     wget \
#     xorg-dev \
#     zsh \
#     gedit \
#     ros-humble-librealsense2* \ 
#     ros-humble-realsense2-* \
#     net-tools \
#     iputils-ping

RUN pip3 install setuptools
RUN pip3 install geopy
RUN pip3 install pymap3d

# Install python3-rosdep and initialize rosdep
# RUN apt-get update && \
#     apt-get install -y python3-rosdep && \
#     rosdep init && \
#     rosdep update

# Install ROS dependencies from a specified path
# RUN rosdep install -i --from-path src --rosdistro humble --skip-keys=librealsense2 -y

# RUN wget https://github.com/openrr/urdf-viz/releases/download/v0.38.2/urdf-viz-x86_64-unknown-linux-gnu.tar.gz && \
#     tar -xvzf urdf-viz-x86_64-unknown-linux-gnu.tar.gz -C /usr/local/bin/ && \
#     chmod +x /usr/local/bin/urdf-viz && \
#     rm -f urdf-viz-x86_64-unknown-linux-gnu.tar.gz

# RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
#     -p git \
#     -p https://github.com/zsh-users/zsh-autosuggestions \
#     -p https://github.com/zsh-users/zsh-completions

# RUN gem install tmuxinator && \
#     wget https://raw.githubusercontent.com/tmuxinator/tmuxinator/master/completion/tmuxinator.zsh -O /usr/local/share/zsh/site-functions/_tmuxinator

RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# # 추가 패키지 설치
# RUN apt-get update && apt-get install -y \
#     build-essential \
#     cmake \
#     libeigen3-dev \
#     libceres-dev \
#     python3-pip

# COPY ${HYUNDAI_AMR_ROOT} /workspace
# RUN mkdir /workspace
ARG HOST_USERNAME=$(hostname)
ARG UID=1000
ENV USER=${HOST_USERNAME}
RUN useradd -u $UID -ms /bin/bash $USER | echo $USER

# RUN chown -R ${USER}:${USER} /workspace
# RUN chmod 755 /workspace

# RUN echo "export DISABLE_AUTO_TITLE=true" >> ~/.bashrc
RUN echo 'LC_NUMERIC="en_US.UTF-8"' >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /amr_ws/install/setup.bash" >> ~/.bashrc
# RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

RUN echo 'alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"' >> ~/.bashrc
RUN echo 'alias cbuild="colcon build --symlink-install"' >> ~/.bashrc
# RUN echo 'alias ssetup="source ./install/local_setup.zsh"' >> ~/.bashrc
# RUN echo 'alias cyclone="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"' >> ~/.bashrc
# RUN echo 'alias fastdds="export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"' >> ~/.bashrc
# RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
# RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc

# RUN echo "autoload -U bashcompinit" >> ~/.bashrc
# RUN echo "bashcompinit" >> ~/.bashrc
# RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> ~/.bashrc
# RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> ~/.bashrc

RUN echo "export ROS_DOMAIN_ID=97" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/" >> ~/.bashrc
RUN echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}" >> ~/.bashrc

# ENV PATH=/usr/local/cuda/bin${PATH:+:${PATH}}

COPY "${HYUNDAI_AMR_ROOT}/operations/scripts/entrypoint.sh" /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]