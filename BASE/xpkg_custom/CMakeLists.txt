cmake_minimum_required(VERSION 3.0.2)
project(xpkg_custom)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  tf
  std_msgs
  xpkg_msgs
)


catkin_package(

)


add_executable(xnode_custom
src/xnode_custom.cpp
src/ROS_INF/ros_interface_custom.cpp)

add_dependencies(xnode_custom xpkg_msgs_generate_messages_cpp)


include_directories(
include
include/ROS_INF
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(xnode_custom
   ${catkin_LIBRARIES}
)


