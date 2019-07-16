#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [ -z "$ROS2_SETUP_PATH" ]
then
    if [ -z "$ROS_DISTRO" ]
    then 
      echo "\$ROS2_SETUP_PATH and \ROS_DISTRO are empty. Is ROS installed? If so, define the variable \$ROS2_SETUP_PATH pointing to the setup.sh script."
      exit 1
    fi

    echo "Sourcing /opt/ros/$ROS_DISTRO/setup.bash"
    source /opt/ros/$ROS_DISTRO/setup.bash 
    
else
    echo "Sourcing $ROS2_SETUP_PATH"
    source $ROS2_SETUP_PATH 
fi

touch $SCRIPT_DIR/bin/COLCON_IGNORE

# Touching the CMakeLists file to be sure we are considering all the new messages
touch $SCRIPT_DIR/src/main/messages/ihmc_interfaces/controller_msgs/CMakeLists.txt

cd $SCRIPT_DIR

echo "Generating C++ interfaces. This may take a while."
colcon build --symlink-install

retVal=$?
if [ $retVal -eq 0 ]; then
    echo "Done!"
    echo "[HINT] Remember to source the file $SCRIPT_DIR/install/setup.bash"
fi

