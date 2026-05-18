FROM dustynv/l4t-pytorch:r36.4.0

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV UHD_VERSION=v4.9.0.0
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV PYTHONPATH=/usr/local/lib/python3/dist-packages:/usr/local/lib/python3.10/dist-packages:$PYTHONPATH
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

SHELL ["/bin/bash", "-c"]

# Base dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl gnupg2 lsb-release software-properties-common ca-certificates \
    git cmake build-essential pkg-config \
    libboost-all-dev libusb-1.0-0-dev \
    python3-dev python3-pip python3-setuptools \
    python3-numpy python3-mako python3-requests python3-pyqtgraph\
    doxygen \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
    
RUN pip install "ultralytics<=8.4" $(pip freeze | grep numpy) -i https://pypi.org/simple

# ROS 2 Humble repo
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Build/install UHD 4.9 from source
RUN git clone --branch ${UHD_VERSION} --depth 1 https://github.com/EttusResearch/uhd.git /tmp/uhd && \
    cd /tmp/uhd/host && \
    mkdir build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DENABLE_TESTS=OFF \
      -DENABLE_EXAMPLES=ON \
      -DENABLE_PYTHON_API=ON && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/uhd

# Download UHD FPGA/images
RUN uhd_images_downloader || true

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
# Set the DDS middleware to cyclone
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

RUN mkdir -p /home/rf_ws/src

CMD ["/bin/bash"]
