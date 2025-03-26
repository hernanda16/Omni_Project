cmake_minimum_required(VERSION 3.0.2)
project(device)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor src/motor.cpp src/pid_controller.cpp)
add_dependencies(motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(motor
  ${catkin_LIBRARIES}
)