ARG ISAAC_VERSION="4.5.0"
# expects context to be the root of the repository, i.e. AirStack/. this is so we can access AirStack/ros_ws/
FROM nvcr.io/nvidia/isaac-sim:${ISAAC_VERSION}
ARG ISAAC_VERSION
WORKDIR /isaac-sim

# isaac's ros2 launch run_isaacsim.launch.py hardcodes to search in this path, so we have to put the executables here
RUN mkdir -p /root/.local/share/ov/pkg/
RUN ln -s /isaac-sim /root/.local/share/ov/pkg/isaac_sim-${ISAAC_VERSION}
# allows us to run isaac-sim as root
ENV OMNI_KIT_ALLOW_ROOT=1
ENV ISAACSIM_PYTHON=/isaac-sim/python.sh
ENV ISAAC_PATH="/isaac-sim"

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    unzip \
    tmux \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN set -eux; \
       key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME"

# setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

# Remove ALL third-party repositories and fix libbrotli issue
RUN rm -rf /etc/apt/sources.list.d/* && \
    echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6 && \
    apt-get install -f -y

# Install basic dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    cmake \
    emacs \
    vim \
    nano \
    tmux \
    tmuxp \
    gdb \
    xterm \
    tree \
    less \
    htop \
    jq \
    libfreetype6-dev \
    libfontconfig1-dev \
    python3-pip \
    python3-scipy \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-dev-tools \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-humble-tf2-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-py \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-topic-tools \
    ros-humble-grid-map \
    ros-humble-domain-bridge \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-urdf \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-controller-manager \
    ros-humble-ackermann-msgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized"
RUN rosdep update

ARG isaac_dir_name="IsaacSim-ros_workspaces-IsaacSim-${ISAAC_VERSION}"
# Install Isaac Sim ROS2 workspace, skip dependency resolution with rosdep and build anyway
RUN cd /tmp/ && \
    curl -L -O https://github.com/isaac-sim/IsaacSim-ros_workspaces/archive/refs/tags/IsaacSim-${ISAAC_VERSION}.zip && \
    unzip IsaacSim-${ISAAC_VERSION}.zip && \
    mv ${isaac_dir_name}/humble_ws /humble_ws && \
    cd /humble_ws && \
    # Skip dependency resolution and build anyway
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /tmp/IsaacSim-${ISAAC_VERSION}.zip /tmp/${isaac_dir_name}

WORKDIR /isaac-sim

# Install Boston Dynamics Spot SDK packages (including choreography protos)
RUN python3 -m pip install \
    bosdyn-client==4.1.1 \
    bosdyn-mission==4.1.1 \
    bosdyn-choreography-client==4.1.1 \
    bosdyn-orbit==4.1.1 \
    bosdyn-choreography-protos==4.1.1

# isaac's ros2 launch run_isaacsim.launch.py hardcodes to search in this path
RUN mkdir -p /root/.local/share/ov/pkg/ && ln -s /isaac-sim /root/.local/share/ov/pkg/isaac-sim-${ISAAC_VERSION}

COPY docker/fastdds.xml /isaac-sim/fastdds.xml

# Cleanup
RUN apt-get purge -y git && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*