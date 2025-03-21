cmake_minimum_required(VERSION 3.0.2)
project(device)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  serial
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor src/motor.cpp)
target_link_libraries(motor ${catkin_LIBRARIES})

add_executable(arduino src/arduino.cpp)
target_link_libraries(arduino ${catkin_LIBRARIES})