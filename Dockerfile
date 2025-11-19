FROM ros:jazzy
WORKDIR /EECE5554

# install apt-utils, will get an error otherwise
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    socat \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# copy source files
COPY ./rpi_workspace /EECE5554/rpi_workspace
