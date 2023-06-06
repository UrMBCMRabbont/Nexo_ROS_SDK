cmake_minimum_required(VERSION 3.0.2)
project(xpkg_comm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  xpkg_msgs
)

catkin_package(

)

add_executable(xnode_comm
src/xnode_comm.cpp
src/ROS_INF/ros_interface.cpp
src/COMM_INF/comm_inf_com.cpp
src/COMM_INF/comm_inf_tcp.cpp
src/FUNC/comm_func.cpp
)

add_dependencies(xnode_comm xpkg_msgs_generate_messages_cpp)

include_directories(
include
include/LIB_JSON
include/ROS_INF
include/COMM_INF
include/FUNC
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(xnode_comm
   ${catkin_LIBRARIES}
 )



