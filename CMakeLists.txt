cmake_minimum_required(VERSION 3.1.3)
project(leo_driving)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  geometry_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
  pcl_conversions
  pcl_ros
  nodelet
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs sensor_msgs tf pcl_conversions nodelet
  DEPENDS PCL 
)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
)

add_library(aisle_detect_node src/aise_detect/aisle_detect_node.cpp)

## Compile as C++14, supported in ROS Kinetic and newer
# set_property(TARGET invz_player PROPERTY CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions( -march=native )

#add_subdirectory(src/lib)

add_executable(leo_driving_node src/cmd_vel_test.cpp src/serial.cpp)
target_link_libraries(leo_driving_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(obs_avoiding_node src/obs_avoid.cpp)
target_link_libraries(obs_avoiding_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

