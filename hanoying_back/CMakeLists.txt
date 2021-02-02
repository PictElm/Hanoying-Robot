cmake_minimum_required(VERSION 3.0.2)
project(hanoying_back)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  genmsg
  actionlib_msgs
  actionlib
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GameRaw.msg
  Disk.msg
  Tower.msg
  GameState.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  GameMove.action
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GameSolve.srv
  GameSolveValidate.srv
  GameSolveList.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hanoying_back
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
