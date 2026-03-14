ARG ROS_BASE_IMAGE=osrf/ros:humble-desktop
ARG PANGOLIN_REF=master
ARG USERNAME=lubobot
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG ROSDEP_SKIP_KEYS="libcrypto boost_serialization"

FROM ${ROS_BASE_IMAGE} AS humble-dev-deps

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
    gosu \
    libeigen3-dev \
    libopencv-dev \
    libboost-system-dev \
    libboost-serialization-dev \
    libssl-dev \
    libglew-dev \
    libepoxy-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libxkbcommon-x11-dev \
    libvulkan1 \
    mesa-vulkan-drivers \
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
    vulkan-tools \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update --rosdistro humble

FROM humble-dev-deps AS ros-workspace-deps

ARG ROSDEP_SKIP_KEYS

COPY docker/rosdep-manifests/ /tmp/rosdep-manifests/

RUN if [ -d /tmp/rosdep-manifests ] && [ "$(find /tmp/rosdep-manifests -name package.xml | wc -l)" -gt 0 ]; then \
      apt-get update && \
      rosdep install -r --from-paths /tmp/rosdep-manifests --ignore-src -y --rosdistro humble --skip-keys "${ROSDEP_SKIP_KEYS}" && \
      rm -rf /var/lib/apt/lists/*; \
    fi

FROM humble-dev-deps AS pangolin-builder

ARG PANGOLIN_REF

RUN git clone --recursive --depth 1 --branch "${PANGOLIN_REF}" https://github.com/stevenlovegrove/Pangolin /tmp/Pangolin && \
    cmake -S /tmp/Pangolin -B /tmp/Pangolin/build -GNinja && \
    cmake --build /tmp/Pangolin/build -j"$(nproc)" && \
    DESTDIR=/tmp/pangolin-install cmake --install /tmp/Pangolin/build && \
    rm -rf /tmp/Pangolin

FROM ros-workspace-deps

ARG USERNAME
ARG USER_ID
ARG GROUP_ID

COPY --from=pangolin-builder /tmp/pangolin-install/usr/local/ /usr/local/

RUN ldconfig

RUN if getent group "${GROUP_ID}" >/dev/null; then \
      true; \
    else \
      groupadd -g "${GROUP_ID}" "${USERNAME}"; \
    fi && \
    useradd -m -s /bin/bash -u "${USER_ID}" -g "${GROUP_ID}" -o "${USERNAME}" && \
    mkdir -p /workspaces && \
    chown -R "${USER_ID}:${GROUP_ID}" /workspaces

COPY docker/humble/entrypoint.sh /usr/local/bin/lubobot-humble-entrypoint
RUN chmod +x /usr/local/bin/lubobot-humble-entrypoint

ENV USER=${USERNAME}
ENV HOME=/home/${USERNAME}
WORKDIR /workspaces/ros2_ws

ENTRYPOINT ["/usr/local/bin/lubobot-humble-entrypoint"]
CMD ["bash"]
