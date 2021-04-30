cmake_minimum_required(VERSION 3.0.2)
project(chessboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  chess_msgs
  actionlib
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chessboard
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
