cmake_minimum_required(VERSION 3.0.2)
project(quadrotors_landing_vision)

find_package(catkin REQUIRED COMPONENTS
    rospy
    gazebo_ros
    gazebo_msgs
    message_generation
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_python_setup()

add_message_files(
    FILES
        UWBRange.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)
 
catkin_package(
    DEPENDS
        rospy
        gazebo_ros
        gazebo_msgs
        message_generation
)

## Install Python scripts as ROS executables (sets +x at install time)
catkin_install_python(
  PROGRAMS
    scripts/drone/drone_fsm.py
    scripts/drone/landing_controller.py
    scripts/positioning/aruco_estimator.py
    scripts/positioning/uwb_lqr_vanc_estimator.py
    scripts/sensor_sim/uwb_simulator.py
    scripts/ugv/ugv_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install resources
install(DIRECTORY launch config worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

