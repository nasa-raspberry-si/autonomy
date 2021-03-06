cmake_minimum_required(VERSION 2.8.3)
project(rs_autonomy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure # for clearing faults
  roscpp
  rospy
  roslib
  actionlib
  actionlib_msgs
  geometry_msgs
  ow_lander
  ow_faults_detection
  ow_plexil
  message_generation
  std_msgs
)

## Setup the installation of python scripts and modules
catkin_python_setup()

add_message_files(
  FILES
  CurrentTask.msg
  NextTask.msg
  PlannerInstruction.msg
  AdaptationInstruction.msg
  VibrationLevel.msg
  EarthInstruction.msg
  ArmFault.msg
  HighLevelPlan.msg
)

add_service_files(
  FILES
  RTInfoMaintenanceInstruction.srv  # maintain runtime info
  ModelUpdateInstruction.srv
  ArmFaultConfig.srv
  TaskPlanning.srv
  PlanTranslation.srv
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
  CATKIN_DEPENDS roscpp roslib rospy ow_lander actionlib_msgs geometry_msgs ow_faults_detection ow_plexil message_runtime
  CFG_EXTRAS
)


###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

#############
## INSTALL ##
#############


# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   scripts/knowledge_management_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
   scripts/fault_management_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
   scripts/task_planning_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
   scripts/plan_translation_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

