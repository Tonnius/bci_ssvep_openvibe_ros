cmake_minimum_required(VERSION 2.8.3)
project(franka_openvibe_move)

#message (status EEEEEEEESHWRTJW>>   ${catkin_LIBRARIES} )

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  moveit_ros_planning_interface
  franka_gripper
  kinova_driver
  kinova_arm_moveit_demo
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(panda_movegroup src/panda_movegroup.cpp)
target_link_libraries(panda_movegroup ${catkin_LIBRARIES})
#add_dependencies(panda_movegroup pr2_movegroup_generate_messages_cpp)

