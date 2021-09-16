cmake_minimum_required(VERSION 3.0.2)
project(smb_highlevel_controller)

# use C++11
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}
  src/smb_highlevel_controller_node.cpp
  src/SmbHighlevelController.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables for installation
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)