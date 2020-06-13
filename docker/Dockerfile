FROM osrf/ros:kinetic-desktop-full

ARG uid
LABEL maintainer="Alexis Pojomovsky"

# Setup environment
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV \
  LANG=en_US.UTF-8 \
  DEBIAN_FRONTEND=noninteractive \
  TERM=xterm

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y && \
    apt-get install --no-install-recommends -y \
      avr-libc \
      avrdude \
      binutils-avr \
      evemu-tools \
      evtest \
      gcc-avr \
      inputattach \
      joystick \
      libcaca0 \
      libevemu3 \
      libftdi1 \
      libgl1-mesa-dri \
      libgl1-mesa-glx \
      libsdl1.2debian \
      libslang2 \
      python-catkin-tools \
      python-rosdep \
      python-rosinstall \
      python-serial \
      ros-kinetic-joy \
      ros-kinetic-joy-teleop \
      ros-kinetic-teleop-tools-msgs \
      ros-kinetic-teleop-tools-msgs \
      sudo \
      tmux \
      xserver-xorg-video-intel && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/share/doc

# Create a user with passwordless sudo
RUN adduser --gecos "Development User" --disabled-password -u ${uid} dev
RUN adduser dev sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Reduce roslaunch timeout
RUN \
  sudo sed -i 's/_TIMEOUT_SIGTERM = 2.0/_TIMEOUT_SIGTERM = 0.5/g' /opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py && \
  sudo sed -i 's/_TIMEOUT_SIGINT  = 15.0/_TIMEOUT_SIGINT  = 1.0/g' /opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py

# Add user permisions to use dialout
RUN usermod -a -G dialout dev

USER dev
RUN echo ". /opt/ros/kinetic/setup.bash" >> /home/dev/.bashrc

RUN rosdep update --include-eol-distros
RUN sudo apt-get update


WORKDIR /catkin_ws/
RUN sudo chown -R dev:dev /catkin_ws
CMD tmux
