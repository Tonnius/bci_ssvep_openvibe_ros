#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <time.h>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

class RobotMover {
private:

public:
    RobotMover() {};
    void moveToHome();
};
