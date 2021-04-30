cmake_minimum_required(VERSION 3.0.2)
project(chess_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  actionlib_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GameCmd.msg
  PlayerInfo.msg
  UIin.msg
  UIout.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SendCmd.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  LobbyRequest.action
  MovePiece.action
  QuickMatch.action
  Rematch.action
  WaitCmd.action
  WaitReady.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chess_msgs
  CATKIN_DEPENDS std_msgs actionlib_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
