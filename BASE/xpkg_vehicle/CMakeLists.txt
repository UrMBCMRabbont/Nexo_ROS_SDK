cmake_minimum_required(VERSION 3.0.2)
project(xpkg_vehicle)

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

add_executable(xnode_vehicle
src/xnode_vehicle.cpp
src/FUNC/vehicle_func_odom.cpp
src/FUNC/vehicle_func.cpp
src/LIB_FILE/lib_file_ini.cpp
src/ROS_INF/ros_interface.cpp
)

add_dependencies(xnode_vehicle xpkg_msgs_generate_messages_cpp)

include_directories(
include
include/FUNC
include/LIB_FILE
include/LIB_JSON
include/ROS_INF
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(xnode_vehicle
   ${catkin_LIBRARIES}
 )

