cmake_minimum_required(VERSION 3.0.2)
project(lite6_labauto)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  xarm_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MovePose.msg
  LabwareOBB.msg
  )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  BeakerMani.srv
  pHMeasure.srv
  PipetteDo.srv
  TweezersDraw.srv
  PipetteDoTwice.srv
  )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_generation message_runtime std_msgs geometry_msgs#xarm_msgs
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############
catkin_install_python(PROGRAMS
   scripts/robot_init.py
   scripts/robot_move.py
   scripts/robot_disable.py
   #scripts/pipette_motor_serial.py
   scripts/gripper_serial.py
   scripts/pH_serial.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
