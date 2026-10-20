cmake_minimum_required(VERSION 2.8.3)
project(b_turnbody_node)

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
  spark_base
)

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
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

add_executable(b_turnbody_actionlib src/turn_body_actionlib.cpp)
add_dependencies(b_turnbody_actionlib ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(b_turnbody_actionlib ${catkin_LIBRARIES} )#if u want to play sound and asr, u need to add "nx_voioce and "msc".

add_executable(turnbody_client_node src/test_turnbody_client.cpp)
add_dependencies(turnbody_client_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(turnbody_client_node ${catkin_LIBRARIES} )#if u want to play sound and asr, u need to add "nx_voioce and "msc".


#############
## Install ##
#############
 
## Mark executables and/or libraries for installation
 install(TARGETS b_turnbody_actionlib turnbody_client_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
 
 install(DIRECTORY
  #  launch
   cfg
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
