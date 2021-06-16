cmake_minimum_required(VERSION 3.1.3)
project(tunnel_driving)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

#### macro parameter
## debug
# add_definitions(-DDEBUG)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  geometry_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
  nodelet
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS laser_geometry geometry_msgs roscpp std_msgs sensor_msgs tf tf2_sensor_msgs pcl_conversions nodelet
  LIBRARIES tunnel_driving
  DEPENDS PCL 
)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
)

add_library(aisle_detect_node
  src/aisle_detect/aisle_detect_node.cpp)
target_link_libraries(aisle_detect_node ${CATKIN_LIBRARIES} ${PCL_LIBRARIES})

add_library(localization_node
  src/localization/localization_node.cpp)
target_link_libraries(localization_node ${CATKIN_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS localization_node
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS aisle_detect_node
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})