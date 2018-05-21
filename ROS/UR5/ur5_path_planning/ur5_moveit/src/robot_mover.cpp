#include "ur5_moveit/robot_mover.h"

void RobotMover::moveToHome() {
  moveit_msgs::DisplayTrajectory display_trajectory;

  moveit::planning_interface::MoveGroupInterface group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  std::vector <geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose = group.getCurrentPose("ee_link").pose;
  group.setNamedTarget("home");

  ROS_INFO("Moving");
  group.execute(my_plan);
  group.move();
  sleep(5.0);
}