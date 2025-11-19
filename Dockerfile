FROM ros:jazzy
WORKDIR /EECE5554

# install dependencies
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    apt-utils \
    socat \
    python3-serial \
    python3-opencv \
    libboost-python-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# copy source files
COPY ./rpi_workspace /EECE5554/rpi_workspace

# setup cv_bridge package
WORKDIR /EECE5554/cv_workspace/src
RUN git clone https://github.com/ros-perception/vision_opencv.git -b rolling && \
    . /opt/ros/jazzy/setup.sh && \
    cd .. && \
    colcon build --symlink-install

# setup ros packages
WORKDIR /EECE5554/rpi_workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# entrypoint stuff
WORKDIR /EECE5554
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

# for once the container starts
CMD ["bash"]
