ARG ROS_BASE_IMAGE=osrf/ros:humble-desktop
ARG PANGOLIN_REF=master

FROM ${ROS_BASE_IMAGE} AS pangolin-builder

ENV DEBIAN_FRONTEND=noninteractive

ARG PANGOLIN_REF

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    pkg-config \
    python3-dev \
    python3-wheel \
    libepoxy-dev \
    libglew-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff5-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libdc1394-dev \
    libopenexr-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    xorg-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --recursive --depth 1 --branch "${PANGOLIN_REF}" https://github.com/stevenlovegrove/Pangolin /tmp/Pangolin && \
    cmake -S /tmp/Pangolin -B /tmp/Pangolin/build -GNinja && \
    cmake --build /tmp/Pangolin/build -j"$(nproc)" && \
    DESTDIR=/tmp/pangolin-install cmake --install /tmp/Pangolin/build && \
    rm -rf /tmp/Pangolin


FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash

RUN apt-get update && apt-get install -y \
    bash-completion \
    build-essential \
    cmake \
    git \
    ninja-build \
    curl \
    ca-certificates \
    pkg-config \
    python3-colcon-common-extensions \
    python3-dev \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-wheel \
    python3-natsort \
    libeigen3-dev \
    libopencv-dev \
    libboost-system-dev \
    libboost-serialization-dev \
    libssl-dev \
    libglew-dev \
    libepoxy-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff5-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    libdc1394-dev \
    libopenexr-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    xorg-dev \
    xauth \
    mesa-utils \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-proc \
    ros-humble-vision-opencv \
    ros-humble-rviz2 \
    ros-humble-usb-cam \
    ros-humble-camera-calibration \
    && rm -rf /var/lib/apt/lists/*

COPY --from=pangolin-builder /tmp/pangolin-install/usr/local/ /usr/local/

RUN ldconfig && \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update --rosdistro humble

RUN useradd -ms /bin/bash -u 1000 dev && \
    mkdir -p /workspaces && \
    chown -R dev:dev /workspaces

COPY docker/humble/entrypoint.sh /usr/local/bin/lubobot-humble-entrypoint
RUN chmod +x /usr/local/bin/lubobot-humble-entrypoint

USER dev
WORKDIR /workspaces/lubobot

ENTRYPOINT ["/usr/local/bin/lubobot-humble-entrypoint"]
CMD ["bash"]
