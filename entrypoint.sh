#!/bin/bash
# tells the shell to exit if any command returns a non-zero exit status
set -e

# source all needed setup scripts
source /opt/ros/jazzy/setup.sh
source /EECE5554/cv_workspace/install/setup.sh
source /EECE5554/rpi_workspace/install/setup.sh

# run the command passed through on the command line (if included)
exec "$@"
