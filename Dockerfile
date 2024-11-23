FROM osrf/ros:noetic-desktop-full

# Install packages
# - default-jdk so it can execute ontologies
RUN apt-get update && apt-get install -y \
    bash-completion \                             
    default-jdk \
    nano \
    python3-argcomplete \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Install python project dependencies
COPY requirements.txt /requirements.txt
RUN pip3 install -r requirements.txt && rm requirements.txt

# Source the ros enviroment every time a new root or ros user shell is opened
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

# Set up and initialize the /catwin_ws workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc

# # These lines are automatically inherited from the base image
# ENTRYPOINT [ "/bin/bash", "/ros_entrypoint.sh" ]
# CMD [ "bash" ]