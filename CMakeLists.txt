cmake_minimum_required(VERSION 3.0.2)
project(rrt_star_planner)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_geometry_msgs
  message_generation
  barracuda_msgs
)

## System dependencies
find_package(PCL REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PlanPath.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  barracuda_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    visualization_msgs 
    pcl_ros 
    pcl_conversions 
    tf2 
    tf2_geometry_msgs 
    message_runtime
    barracuda_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(rrt_star_planner_node 
  src/rrt_star_planner_node.cpp
  src/rrt_star_planner.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(rrt_star_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(rrt_star_planner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(TARGETS rrt_star_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
