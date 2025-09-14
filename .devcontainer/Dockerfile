FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    # GUI defaults for X11 inside container
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1

# OS + build deps (ROS tools + native libs + GUI/GL + init)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl \
    python3-pip python3-dev python3-rosdep \
    ros-noetic-catkin ros-noetic-rviz ros-noetic-tf \
    ros-noetic-image-transport ros-noetic-cv-bridge \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions \
    libopencv-dev libpcl-dev qtbase5-dev libqt5svg5-dev \
    libvtk7-dev libopenni2-dev libfreenect-dev \
    # GUI/GL runtime so rtabmap_viz / rviz can display
    x11-apps mesa-utils libgl1-mesa-dri libgl1 \
    # tiny init for clean PID1
    dumb-init \
 && rm -rf /var/lib/apt/lists/*

# Init rosdep (ok as root inside container)
RUN rosdep init || true && rosdep update

# Python libs (CPU wheels; avoid pip opencv to keep cv_bridge happy)
# Pin torch/torchvision to CPU wheels
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
      torch==2.2.2+cpu \
      torchvision==0.17.2+cpu \
      --index-url https://download.pytorch.org/whl/cpu && \
    python3 -m pip install \
      ultralytics==8.3.0 \
      numpy==1.24.4 \
      scipy==1.10.1 \
      matplotlib==3.5.3 \
      absl-py==1.4.0 \
      attrs==23.1.0 \
      packaging==23.1 \
      pillow==9.5.0 \
      python-dateutil==2.8.2 \
      six==1.16.0

# Workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Build only what we need locally (no rtabmap_ros here)
RUN git clone --depth 1 https://github.com/ros-drivers/freenect_stack.git

# RTAB-Map (ROS Noetic) from apt – provides core + ROS wrapper
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rtabmap-ros \
 && rm -rf /var/lib/apt/lists/*

# Resolve deps for sources in /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN . /opt/ros/noetic/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Build catkin workspace
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"

# Keep container alive; devcontainers can override this when overrideCommand=true
CMD ["dumb-init","sleep","infinity"]
