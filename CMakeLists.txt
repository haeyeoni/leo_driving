cmake_minimum_required(VERSION 2.8.3)
project(leo_driving)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        tf
        nodelet
        tf2_sensor_msgs
        pcl_conversions
        laser_geometry
        )
find_package(PCL REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        LIBRARIES LocalizationNode AisleDetectNode ObstaclesNode CmdPublishNode
        CATKIN_DEPENDS nodelet roscpp sensor_msgs tf geometry_msgs pcl_conversions laser_geometry
        DEPENDS PCL
)

#add_executable(rp_driving_node src/rp_node.cpp )
#target_link_libraries(rp_driving_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(LocalizationNode src/localization_node.cpp)
add_dependencies(LocalizationNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(LocalizationNode ${catkin_LIBRARIES})

add_library(AisleDetectNode src/aisle_detect_node.cpp)
add_dependencies(AisleDetectNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AisleDetectNode ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(ObstaclesNode src/obstacles_node.cpp)
add_dependencies(ObstaclesNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ObstaclesNode ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(CmdPublishNode src/cmd_publish_node.cpp)
add_dependencies(ObstaclesNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ObstaclesNode ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(${PROJECT_NAME}_nodelets
  src/localization_node.cpp
  src/aisle_detect_node.cpp
  src/obstacles_node.cpp
  src/cmd_publish_node.cpp)

target_link_libraries(${PROJECT_NAME}_nodelets LocalizationNode AisleDetectNode ObstaclesNode CmdPublishNode)


install(TARGETS LocalizationNode AisleDetectNode ObstaclesNode CmdPublishNode
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
