# build stage with base tools
FROM ros:jazzy AS build
WORKDIR /EECE5554
SHELL ["/bin/bash", "-c"]

# copy source files
COPY ./rpi_workspace /EECE5554/rpi_workspace

# install dependencies
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    apt-utils \
    socat \
    && rm -rf /var/lib/apt/lists/*


# build stage with python dependencies
FROM build AS python_setup
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    python3-serial \
    python3-opencv \
    libboost-python-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*


# setup ros packages
FROM python_setup AS ros_setup
WORKDIR /EECE5554/rpi_workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install


# setup cv_bridge package
FROM python_setup AS cv_setup
WORKDIR /EECE5554/cv_workspace/src
RUN git clone https://github.com/ros-perception/vision_opencv.git -b rolling && \
    . /opt/ros/jazzy/setup.sh && \
    cd .. && \
    colcon build --symlink-install


# finalization
FROM python_setup AS finalize
COPY --from=ros_setup /EECE5554/rpi_workspace /EECE5554/rpi_workspace
COPY --from=cv_setup /EECE5554/cv_workspace /EECE5554/cv_workspace
WORKDIR /EECE5554
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

# for once the container starts
CMD ["bash"]
