cmake_minimum_required(VERSION 2.8.3)
project(turtlesim_openloop_control)

find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(turtlesim_openloop_control_node src/turtlesim_openloop_control_node.cpp)

target_link_libraries(turtlesim_openloop_control_node
  ${catkin_LIBRARIES}
)

install(PROGRAMS
  scripts/turtlesim_openloop_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
