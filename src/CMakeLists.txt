cmake_minimum_required(VERSION 2.8.3)
project(leo_drive)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++14 -o3")

        #add_include_directories("/opt/ros/melodic/include")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  std_msgs
  genmsg
  laser_geometry
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  visualization_msgs
  std_msgs
  genmsg
  laser_geometry
  sensor_msgs
  geometry_msgs
)

include_directories(
        "/usr/include/eigen3"
        "/opt/ros/melodic/include"
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)



add_executable(leo_drive 
  cmd_vel_test.cpp
  cmd_vel_test.h 
)

target_link_libraries(leo_drive
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
)

