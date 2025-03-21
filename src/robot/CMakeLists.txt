cmake_minimum_required(VERSION 3.0.2)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  roslib
  )

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/robot_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})