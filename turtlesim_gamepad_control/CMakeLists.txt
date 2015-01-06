cmake_minimum_required(VERSION 2.8.3)
project(turtlesim_gamepad_control)

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs geometry_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlesim_gamepad_control_node src/turtlesim_gamepad_control_node.cpp)

target_link_libraries(turtlesim_gamepad_control_node
  ${catkin_LIBRARIES}
)

install(PROGRAMS
  scripts/turtlesim_gamepad_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
