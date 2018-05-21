
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

#include <kinova_driver/kinova_ros_types.h>
#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_api.h"

#include <ros/ros.h>
#include <cstdlib>
#include <pick_place.h>
#include <ros/console.h>
#include <kinova_driver/gripper_command_action_server.h>
#include <franka_control/SetFullCollisionBehavior.h>

#include <unistd.h>
#include <franka_control/ErrorRecoveryAction.h>
#include <franka_control/ErrorRecoveryActionGoal.h>
#include <franka_msgs/Errors.h>
#include <franka_control/ErrorRecoveryGoal.h>


using namespace kinova;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_movegroup");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();




       /*///////////////////////////////////////////////////////////////
       -                                                               -
       -                ////  Code for Franka-Kinova ////              -
       -                                                               -
       ///////////////////////////////////////////////////////////////*/



        ROS_INFO("This is Franka_Kinova_Code");



        /* This sleep is ONLY to allow Rviz to come up */

        sleep(5.0);



        // Setup
        // ^^^^^
        //
        // The :move_group_interface:`MoveGroup` class can be easily
        // setup using just the name
        // of the group you would like to control and plan for.

        moveit::planning_interface::MoveGroup group("panda_arm_hand");


        // We will use the :planning_scene_interface:`PlanningSceneInterface`
        // class to deal directly with the world.
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // (Optional) Create a publisher for visualizing plans in Rviz.
        ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        moveit_msgs::DisplayTrajectory display_trajectory;



        // Set Franka external force Threshold

        ros::ServiceClient client = node_handle.serviceClient<franka_control::SetFullCollisionBehavior>("/panda/set_full_collision_behavior");

        franka_control::SetFullCollisionBehavior srv;


        srv.request.lower_force_thresholds_acceleration.assign (100.0);
        srv.request.lower_force_thresholds_nominal.assign (100.0);
        srv.request.lower_torque_thresholds_acceleration.assign (100.0);
        srv.request.lower_torque_thresholds_nominal.assign (100.0);

        srv.request.upper_force_thresholds_acceleration.assign (100.0);
        srv.request.upper_force_thresholds_nominal.assign (100.0);
        srv.request.upper_torque_thresholds_acceleration.assign (100.0);
        srv.request.upper_torque_thresholds_nominal.assign (100.0);

        client.call(srv);

        // start kinova action client server

        actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> kinova_client("/m1n6s300_driver/fingers_action/finger_positions", true);


        // wait for the Move action client server to start

        ros::WallDuration(2.0).sleep();

        ROS_INFO("Action server started");


        // set the fingers to open position

        kinova_msgs::SetFingersPositionGoal open;

        open.fingers.finger1 = 0.0;
        open.fingers.finger2 = 0.0;
        open.fingers.finger3 = 0.0;

        kinova_client.sendGoal(open);

        // First Cartesian Path From default Pose to the Grasp Pose

        std::vector<geometry_msgs::Pose> grasp;

        geometry_msgs::Pose target_pose_1;

        target_pose_1.position.x = 0.90196;
        target_pose_1.position.y = -0.023556;
        target_pose_1.position.z = 0.672764;
        target_pose_1.orientation.x = 0.034101;
        target_pose_1.orientation.y = -0.0199791;
        target_pose_1.orientation.z = 0.304773;
        target_pose_1.orientation.w = 0.951604;


        grasp.push_back(target_pose_1); // default position


        target_pose_1.position.x = 0.92892;
        target_pose_1.position.y = 0.15367;
        target_pose_1.position.z = 0.42031;
        target_pose_1.orientation.x = 0.015608;
        target_pose_1.orientation.y = -0.00400;
        target_pose_1.orientation.z = 0.307639;
        target_pose_1.orientation.w = 0.951366;


        grasp.push_back(target_pose_1);  // Point A Grasp Pose


        // We want the cartesian path to be interpolated at a resolution of 1 cm
        // which is why we will specify 0.01 as the max step in cartesian
        // translation.  We will specify the jump threshold as 0.0, effectively
        // disabling it.


        moveit_msgs::RobotTrajectory trajectory_1;
        double fraction_1 = group.computeCartesianPath(grasp,
                                                   0.01,  // eef_step
                                                   0.0,   // jump_threshold
                                                   trajectory_1);

        moveit::planning_interface::MoveGroup::Plan grasp_plan;
        grasp_plan.trajectory_ = trajectory_1;


        ROS_INFO("Visualizing cartesian path one (%.2f%% acheived)",
            fraction_1 * 100.0);

        // Excute first cartesian path

         group.execute(grasp_plan);


        ros::Duration(0.1).sleep();


        // set the fingers to grasp position

        kinova_msgs::SetFingersPositionGoal close;

        close.fingers.finger1 = 6400.0;
        close.fingers.finger2 = 6400.0;
        close.fingers.finger3 = 6400.0;

        kinova_client.sendGoal(close);



        while(!kinova_client.getState().isDone())
        {

        // just looping until it finishes the required action
        ros::Duration(0.1).sleep();

        }


        // Second Cartesian Path , Releasing Pose

        std::vector<geometry_msgs::Pose> release;

        geometry_msgs::Pose target_pose_2;

        target_pose_2.position.x = 1.1986;
        target_pose_2.position.y = 0.003956;
        target_pose_2.position.z = 0.44126;
        target_pose_2.orientation.x = 0.0217006;
        target_pose_2.orientation.y = -0.000956;
        target_pose_2.orientation.z = 0.28334;
        target_pose_2.orientation.w = 0.958775;

        //release.push_back(target_pose_2);  // Point B

        target_pose_2.position.x = 1.048;
        target_pose_2.position.y = 0.0152;
        target_pose_2.position.z = 0.589;
        target_pose_2.orientation.x = 0.515;
        target_pose_2.orientation.y = -0.351;
        target_pose_2.orientation.z = 0.185;
        target_pose_2.orientation.w = 0.759;

        release.push_back(target_pose_2);  // Point C Releasing Pose


        moveit_msgs::RobotTrajectory trajectory_2;
        double fraction_2 = group.computeCartesianPath(release,
                                                   0.01,  // eef_step
                                                   0.0,   // jump_threshold
                                                   trajectory_2);

        moveit::planning_interface::MoveGroup::Plan release_plan;
        release_plan.trajectory_ = trajectory_2;

        ROS_INFO("Visualizing cartesian path two (%.2f%% acheived)",
            fraction_2 * 100.0);

        // Excute Second cartesian path


         group.execute(release_plan);

        ros::Duration(0.1).sleep();

        kinova_client.sendGoal(open);


        while (!kinova_client.getState().isDone());

        {

        // just looping until it finishes the required action
        ros::Duration(0.1).sleep();
        }

        // Go Back to The Default Position when process is finished

        std::vector<geometry_msgs::Pose> end;

        geometry_msgs::Pose target_pose_up;

        target_pose_up.position.x = 1.42744;
        target_pose_up.position.y = -0.0130651;
        target_pose_up.position.z = 0.669737;
        target_pose_up.orientation.x = 0.007800;
        target_pose_up.orientation.y = -0.0102331;
        target_pose_up.orientation.z = 0.289835;
        target_pose_up.orientation.w = 0.956991;

        //end.push_back(target_pose_up);  // go up liitle in Point C



        target_pose_up.position.x = 0.90196;
        target_pose_up.position.y = -0.023556;
        target_pose_up.position.z = 0.672764;
        target_pose_up.orientation.x = 0.034101;
        target_pose_up.orientation.y = -0.0199791;
        target_pose_up.orientation.z = 0.304773;
        target_pose_up.orientation.w = 0.951604;


        end.push_back(target_pose_up);  // Go Back To The Default Position


        moveit_msgs::RobotTrajectory trajectory_3;
        double fraction_3 = group.computeCartesianPath(end,
                                                   0.01,  // eef_step
                                                   0.0,   // jump_threshold
                                                   trajectory_3);

        moveit::planning_interface::MoveGroup::Plan end_plan;
        end_plan.trajectory_ = trajectory_3;

        ROS_INFO("FINISH (%.2f%% acheived)",
            fraction_3 * 100.0);

        // Excute Second cartesian path

         group.execute(end_plan);


    ros::shutdown();
    return 0;
}
