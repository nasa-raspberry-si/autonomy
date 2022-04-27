cmake_minimum_required(VERSION 2.8.3)
project(rs_autonomy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  actionlib
  actionlib_msgs
  geometry_msgs
  ow_lander
  ow_faults
  ow_plexil
  message_generation
  std_msgs
)


add_message_files(
  FILES
  EInfo.msg
  MInfo.msg
  AInfo.msg
  PInfo.msg
)


generate_messages(
  DEPENDENCIES 
  geometry_msgs
  std_msgs
  actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib rospy ow_lander actionlib_msgs geometry_msgs ow_faults ow_plexil message_runtime
  CFG_EXTRAS
)


###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

#############
## INSTALL ##
#############



