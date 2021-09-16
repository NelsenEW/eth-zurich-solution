cmake_minimum_required(VERSION 3.0.2)
project(smb_description)

find_package(catkin REQUIRED COMPONENTS
    xacro
    hector_gazebo_plugins
    urdf
    roslib
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    xacro
    hector_gazebo_plugins
)

add_definitions("--std=c++11")
include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch rviz urdf resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
