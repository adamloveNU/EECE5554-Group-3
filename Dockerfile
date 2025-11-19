FROM ros:jazzy
WORKDIR /EECE5554

# install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    socat \
    python3-serial \
    python3-opencv \
    libboost-python-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# setup cv_bridge package
WORKDIR /EECE5554/cv_bridge/src
RUN git clone https://github.com/ros-perception/vision_opencv.git -b rolling

# copy source files
WORKDIR /EECE5554
COPY ./rpi_workspace /EECE5554/rpi_workspace
