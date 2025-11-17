FROM ros:jazzy
WORKDIR /EECE5554

# install apt-utils, will get an error otherwise
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-serial \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# copy source files
COPY ./ros2_workspace /EECE5554/ros2_workspace
