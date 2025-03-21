cmake_minimum_required(VERSION 3.0.2)
project(comm)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/comm_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})