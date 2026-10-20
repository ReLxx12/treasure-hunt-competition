cmake_minimum_required(VERSION 2.8.3)
project(move_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  common
  actionlib
  actionlib_msgs
  dynamic_reconfigure
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES b_turnbody_node
#  CATKIN_DEPENDS roscpp rospy std_msgs actionlib actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



#############
## Install ##
#############
 

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

