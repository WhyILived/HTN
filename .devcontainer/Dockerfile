# Base image: ROS Noetic desktop
FROM osrf/ros:noetic-desktop-full

# Avoid interactive prompts during install
ENV DEBIAN_FRONTEND=noninteractive

# Update OS and install Python essentials
RUN apt-get update && \
    apt-get install -y python3-pip python3-dev git curl && \
    rm -rf /var/lib/apt/lists/*

# Upgrade pip to latest version
RUN python3 -m pip install --upgrade pip

# Set working directory
WORKDIR /app

# Install Python packages directly (fixed protobuf/mediapipe conflict)
RUN pip3 install --no-cache-dir \
    absl-py==1.4.0 \
    attrs==23.1.0 \
    cffi==1.15.1 \
    contourpy==1.0.7 \
    cycler==0.11.0 \
    flatbuffers==22.12.6 \
    fonttools==4.39.3 \
    jax==0.4.13 \
    jaxlib==0.4.13 \
    kiwisolver==1.4.4 \
    matplotlib==3.5.3 \
    ml_dtypes==0.1.0 \
    numpy==1.24.4 \
    opencv-contrib-python==4.7.0.72 \
    opencv-python==4.7.0.72 \
    opt_einsum==3.3.0 \
    packaging==23.1 \
    pillow==9.5.0 \
    "protobuf<4,>=3.11" \
    pycparser==2.21 \
    pyparsing==3.0.9 \
    python-dateutil==2.8.2 \
    scipy==1.10.1 \
    setuptools==67.8.0 \
    six==1.16.0 \
    sounddevice==0.4.6 \
    mediapipe==0.10.11 \
    ultralytics \
    torch \
    torchvision

# Default command
CMD ["/bin/bash"]
