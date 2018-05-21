cmake_minimum_required(VERSION 2.8.3)
project(ur5_moveit)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        moveit_core
        roscpp
        rospy
        std_msgs
        message_generation
        moveit_ros_planning
        moveit_ros_planning_interface
        pluginlib
        geometric_shapes
        moveit_visual_tools)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur5_moveit
  CATKIN_DEPENDS
    moveit_core
    roscpp
    std_msgs
    message_runtime
    moveit_core
    moveit_ros_planning_interface
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)

add_library(ur5_moveit
        src/robot_mover.cpp)
target_link_libraries(ur5_moveit ${catkin_LIBRARIES})
add_dependencies(ur5_moveit ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)