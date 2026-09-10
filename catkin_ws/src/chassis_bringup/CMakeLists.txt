cmake_minimum_required(VERSION 2.8.3)
project(chassis_bringup)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  dynamic_reconfigure
)

# ulimit -c unlimited
add_compile_options(-g -std=c++11)    #-g -> core dumped


generate_dynamic_reconfigure_options(
  cfg/CalibrateAngular.cfg
  cfg/CalibrateLinear.cfg
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES turn_on_wheeltec_robot
  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf dynamic_reconfigure
  #DEPENDS system_lib
)

# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME})

add_executable(chassis_drive_node src/chassis_drive.cpp src/Quaternion_Solution.cpp)
target_link_libraries(chassis_drive_node ${catkin_LIBRARIES})
