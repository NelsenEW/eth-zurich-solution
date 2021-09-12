cmake_minimum_required(VERSION 3.0.2)
project(smb_gazebo)
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra")

set(PACKAGE_DEPENDENCIES
  gazebo_ros
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    odometry_source_plugin
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    gazebo_ros
    Boost
)

add_library(
  odometry_source_plugin
        src/OdometrySource.cpp
)

add_dependencies(
  odometry_source_plugin
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  odometry_source_plugin
  ${catkin_LIBRARIES}
)


install(TARGETS odometry_source_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
