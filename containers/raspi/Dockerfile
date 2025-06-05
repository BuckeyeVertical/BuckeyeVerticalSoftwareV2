FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

ENV CXXFLAGS="-std=c++14"

# 1) Remove any existing ROS 2 list so we don’t pick up an old key
RUN rm -f /etc/apt/sources.list.d/ros2.list

# 2) Install gnupg2, curl, lsb-release, software-properties-common
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      gnupg2 \
      curl \
      lsb-release \
      software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN rm -f /usr/share/keyrings/ros-archive-keyring.gpg && \
    mkdir -p /usr/share/keyrings && \
    curl -fSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# 4) Add the ROS 2 repository, signed by that key
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install dependencies and Python packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    cmake \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

# Install Python dependencies
RUN pip install --no-cache-dir -U empy==3.3.4 pyros-genmsg setuptools==65.5.1

# Clone and build Micro-XRCE-DDS-Agent
WORKDIR /tmp
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/ && \
    cd / && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

# Install eigen
RUN apt update && apt install -y libeigen3-dev && \
    apt-get install -y ros-humble-rviz2

COPY requirements.txt /requirements.txt
RUN pip install --upgrade pip && pip install -r /requirements.txt

# Install all GStreamer runtimes + dev packages in one go
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      gstreamer1.0-tools \
      gstreamer1.0-alsa \
      gstreamer1.0-plugins-base \
      gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad \
      gstreamer1.0-plugins-ugly \
      gstreamer1.0-libav \
      libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0-dev \
      libgstreamer-plugins-good1.0-dev \
      libgstreamer-plugins-bad1.0-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
# Set working directory for the container
WORKDIR /bv_ws

# Copy the entrypoint script from host to container
COPY entrypoint.sh /entrypoint.sh

# Make entrypoint script executable
RUN chmod +x /entrypoint.sh

# Source ROS environment in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
