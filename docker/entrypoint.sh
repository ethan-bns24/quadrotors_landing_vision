#!/bin/bash

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS  ${ROS_DISTRO}"

# Add PX4 Gazebo modules to ROS_PACKAGE_PATH
if [ -f /quadrotors_landing_vision/PX4-Autopilot/Tools/setup_gazebo.bash ]
then 
  source /quadrotors_landing_vision/PX4-Autopilot/Tools/setup_gazebo.bash /quadrotors_landing_vision/PX4-Autopilot /quadrotors_landing_vision/PX4-Autopilot/build/px4_sitl_default
  echo "Sourced PX4 Gazebo modules"
fi

# Source the base workspace, if built
if [ -f ${ROS_WORKSPACE}/devel/setup.bash ]
then
  source ${ROS_WORKSPACE}/devel/setup.bash
  echo "Sourced ${ROS_WORKSPACE} workspace"
fi

# Add PX4 modules to ROS_PACKAGE_PATH
if [ -f /quadrotors_landing_vision/PX4-Autopilot/Tools/setup_gazebo.bash ]
then 
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/quadrotors_landing_vision/PX4-Autopilot
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/quadrotors_landing_vision/PX4-Autopilot/Tools/sitl_gazebo
fi

# Add project's models to GAZEBO_MODE_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${ROS_WORKSPACE}/src/quadrotors_landing_vision/models

# Execute the command passed into this entrypoint
exec "$@"
