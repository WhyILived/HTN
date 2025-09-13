# Base image: your ROS Noetic desktop
FROM osrf/ros:noetic-desktop-full

# Avoid interactive prompts during install
ENV DEBIAN_FRONTEND=noninteractive

# Update OS and install pip + Python essentials
RUN apt-get update && \
    apt-get install -y python3-pip python3-dev python3-venv git curl && \
    rm -rf /var/lib/apt/lists/*

# Upgrade pip to latest version
RUN python3 -m pip install --upgrade pip

# Install Python packages you need
RUN pip3 install --no-cache-dir ultralytics numpy matplotlib opencv-python torch torchvision

# Set workdir inside container
WORKDIR /app

# Copy your code if you want (optional)
# COPY . /app

# Default command
CMD ["/bin/bash"]
