
#include <moveit/move_group_interface/move_group_interface.h>
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
#include "std_msgs/String.h"

using namespace kinova;

int inDataOpenVibe = 0;
char inputKeyboard = '0';

enum class StateClass
{
    HOME,
    ITEM1,
    ITEM2,
    USER,
    ERROR,
    OKAY,
    NONE
};

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    inDataOpenVibe = std::stoi(msg->data.c_str());
    //ROS_INFO("I heard from openvibe: [%d]", inDataOpenVibe);
}

void keyboardCallback(const std_msgs::String::ConstPtr& msg)
{
    inputKeyboard = msg->data[0];
    //ROS_INFO("I heard from keyboard: [%c]", inputKeyboard);
}

void planAndMove(std::vector<geometry_msgs::Pose> poseVecIn, 
                moveit::planning_interface::MoveGroupInterface &groupRef,
                StateClass &state) {

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = groupRef.computeCartesianPath(poseVecIn,
                                                       0.01,  // eef_step
                                                       0.0,   // jump_threshold
                                                       trajectory);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    // Excute cartesian path and check for errors
    if (!groupRef.execute(plan))
    {
        ROS_INFO("Plan Failed, changing the state to ERROR");
        state = StateClass::ERROR;
    } else {
        state = StateClass::OKAY;
    }
}
//Open or close gripper, argument 'open = true' to open, 'open = false' to close
void setGripper(bool open, actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> &client, bool &gripperState) {
    // grippr open position
    kinova_msgs::SetFingersPositionGoal openPos;
    openPos.fingers.finger1 = 0.0;
    openPos.fingers.finger2 = 0.0;
    openPos.fingers.finger3 = 0.0;

    // gripper close position
    kinova_msgs::SetFingersPositionGoal closePos;
    closePos.fingers.finger1 = 6000.0;
    closePos.fingers.finger2 = 6000.0;
    closePos.fingers.finger3 = 6000.0;

    if (open) {
        //ROS_INFO("OPENING GRIPPER");
        client.sendGoal(openPos);    
    } else {
        //ROS_INFO("CLOSING GRIPPER");
        client.sendGoal(closePos);
    }

    while(!client.getState().isDone()) {
        ros::Duration(0.1).sleep();
    }

    gripperState = open;

}

int main(int argc, char* argv[])
{
    bool useKeyboard;

    bool item1InGripper = false;
    bool item2InGripper = false;

    bool gripperOpenState = true;

    ros::init(argc, argv, "panda_movegroup");
    ros::NodeHandle node_handle;
    ros::Subscriber sub = node_handle.subscribe("chatter", 100, chatterCallback);
    ros::Subscriber sub2 = node_handle.subscribe("panda_movegroup/keyboard", 100, keyboardCallback);
   
    if (node_handle.getParam("/panda_movegroup/use_keyboard", useKeyboard))
    {
      ROS_INFO("Got param: %d", useKeyboard);
    }
    else
    {
      ROS_ERROR("Failed to get param 'use_keyboard'");
      useKeyboard = true;
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("panda_arm_hand");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // (Optional) Create a publisher for visualizing plans in RViz.
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Set Franka external force thresholds
    ros::ServiceClient client = node_handle.serviceClient<franka_control::SetFullCollisionBehavior>("set_full_collision_behavior");
    franka_control::SetFullCollisionBehavior srv;

    srv.request.lower_force_thresholds_acceleration.assign (20.0);
    srv.request.lower_force_thresholds_nominal.assign (20.0);
    srv.request.lower_torque_thresholds_acceleration.assign (20.0);
    srv.request.lower_torque_thresholds_nominal.assign (20.0);
    srv.request.upper_force_thresholds_acceleration.assign (300.0);
    srv.request.upper_force_thresholds_nominal.assign (300.0);
    srv.request.upper_torque_thresholds_acceleration.assign (300.0);
    srv.request.upper_torque_thresholds_nominal.assign (300.0);

    client.call(srv);

    // Set tolrance of movements
    group.setGoalTolerance(0.01);

    // Start kinova gripper action client server
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> kinova_client("/c1n4s300_driver/fingers_action/finger_positions", true);

    // Error recovery action
    ros::Publisher error_pub = node_handle.advertise<franka_control::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 1000);
    franka_control::ErrorRecoveryActionGoal err;

    // Loop rate
    ros::Rate loop_rate(30);
    
    // Initial state machine states
    StateClass state = StateClass::HOME;
    StateClass prevState = StateClass::NONE;
    
    bool debugEnabled = false;

    // Available poses (cartesian)
    geometry_msgs::Pose home;
    home.position.x = 0.963962;
    home.position.y = -0.0894077;
    home.position.z = 0.65493;
    home.orientation.x = -0.033849;
    home.orientation.y = -0.009381;
    home.orientation.z = 0.309561;
    home.orientation.w = 0.950232;

    geometry_msgs::Pose goNearItem1;
    goNearItem1.position.x = 1.25399;
    goNearItem1.position.y = -0.266322;
    goNearItem1.position.z = 0.295;
    goNearItem1.orientation.x = 0.689556;
    goNearItem1.orientation.y = 0.199959;
    goNearItem1.orientation.z = -0.628889;
    goNearItem1.orientation.w = -0.298375;

    geometry_msgs::Pose graspItem1;
    graspItem1.position.x = 1.3364;
    graspItem1.position.y = -0.2755;
    graspItem1.position.z = 0.2906;
    graspItem1.orientation.x = 0.686;
    graspItem1.orientation.y = 0.198;
    graspItem1.orientation.z = -0.6321;
    graspItem1.orientation.w = -0.3009;

    geometry_msgs::Pose goNearItem2;
    goNearItem2.position.x = 1.3182;
    goNearItem2.position.y = -0.0041;
    goNearItem2.position.z = 0.53;
    goNearItem2.orientation.x = -0.0283;
    goNearItem2.orientation.y = 0.0302;
    goNearItem2.orientation.z = 0.8897;
    goNearItem2.orientation.w = 0.4545;

    geometry_msgs::Pose graspItem2; 
    graspItem2.position.x = 1.3251;
    graspItem2.position.y = -0.0008;
    graspItem2.position.z = 0.4099;
    graspItem2.orientation.x = -0.0045;
    graspItem2.orientation.y = 0.0093;
    graspItem2.orientation.z = 0.9261;
    graspItem2.orientation.w = 0.3771;

    geometry_msgs::Pose goToUser;
    goToUser.position.x = 0.9818;
    goToUser.position.y = 0.2082;
    goToUser.position.z = 0.6347;
    goToUser.orientation.x = -0.3823;
    goToUser.orientation.y = -0.5232;
    goToUser.orientation.z = 0.7516;
    goToUser.orientation.w = -0.1231;

    // Open gripper
    setGripper(true, kinova_client, gripperOpenState);

    // Keep running until false
    bool running = true;

    if(useKeyboard == true) {
        ROS_INFO("Please press 'b' for bottle, 't' for tea, 'h' for home, 'u' for user or 'e' for exit");
    } else {
        ROS_INFO("Please make sure openvibe and comm_tcp are running");
    }
    while (ros::ok() && running)
    {
        while (state != StateClass::ERROR && ros::ok() && running)
        {   
            if (inputKeyboard == 'e'){
                running = false;
                ROS_INFO("pressed e for exit");  //CTRL + C also works
                break;
            }
            else if(inDataOpenVibe == 1 || inputKeyboard == 'h') {
                state = StateClass::HOME;
                break;
            }
            else if(inDataOpenVibe == 4 || inputKeyboard == 'u') {
                state = StateClass::USER;
                break;
            }
            else if(inDataOpenVibe == 2 || inputKeyboard == 'b') {
                state = StateClass::ITEM1;
                break;
            }
            else if(inDataOpenVibe == 3 || inputKeyboard == 't') {
                state = StateClass::ITEM2;
                break;
            }
            
            loop_rate.sleep();                 
        };

        switch (state)
        {
            case StateClass::HOME:
            {
                if (debugEnabled == true) ROS_INFO("ENTER HOME");  
                std::vector<geometry_msgs::Pose> poseVec;               
                poseVec.push_back(home);

                planAndMove(poseVec, group, state);
                if (state == StateClass::ERROR) break;             

                prevState = StateClass::HOME;
                if (debugEnabled == true) ROS_INFO("EXIT HOME");  
            }
            break;

            case StateClass::ITEM1:
            {
                if (prevState != StateClass::ITEM1) {
                    if (debugEnabled == true) ROS_INFO("ENTER ITEM1");
                    std::vector<geometry_msgs::Pose> poseVec;

                    // Put item 2 back if it's in the gripper
                    if (item2InGripper) {
                        poseVec.clear();
                        poseVec.push_back(goNearItem2); 
                        poseVec.push_back(graspItem2);
                        // Move to item 2
                        planAndMove(poseVec, group, state);
                        if (state == StateClass::ERROR) break;

                        setGripper(true, kinova_client, gripperOpenState);

                        poseVec.clear();
                        poseVec.push_back(goNearItem2);
                        // Move away a little from item 2
                        planAndMove(poseVec, group, state);
                        if (state == StateClass::ERROR) break;
                    }

                    poseVec.clear();
                    poseVec.push_back(goNearItem1); 
                    poseVec.push_back(graspItem1); 
                    // Move to item 1
                    planAndMove(poseVec, group, state);
                    if (state == StateClass::ERROR) break;  
                    // Grip or release item 1
                    setGripper(item1InGripper, kinova_client, gripperOpenState);
                    item1InGripper = !item1InGripper;
                    
                    prevState = StateClass::ITEM1;
                    if (debugEnabled == true) ROS_INFO("EXIT ITEM1");
                }
               
            }
            break;

            case StateClass::ITEM2:
            {
                if (prevState != StateClass::ITEM2) {
                    if (debugEnabled == true) ROS_INFO("ENTER ITEM2");                
                    std::vector<geometry_msgs::Pose> poseVec;

                    // Put item 1 back if it's in the gripper
                    if (item1InGripper) {
                        poseVec.clear();
                        poseVec.push_back(goNearItem1); 
                        poseVec.push_back(graspItem1);
                        // Move to item 1
                        planAndMove(poseVec, group, state);
                        if (state == StateClass::ERROR) break;

                        setGripper(true, kinova_client, gripperOpenState);

                        poseVec.clear();
                        poseVec.push_back(goNearItem1);
                        // Move away a little from item 1
                        planAndMove(poseVec, group, state);
                        if (state == StateClass::ERROR) break;
                    }
                    
                    poseVec.clear();
                    poseVec.push_back(goNearItem2); 
                    poseVec.push_back(graspItem2);
                    // Move to item 2
                    planAndMove(poseVec, group, state);
                    if (state == StateClass::ERROR) break;   

                    setGripper(item2InGripper, kinova_client, gripperOpenState);
                    item2InGripper = !item2InGripper;

                    prevState = StateClass::ITEM2;
                    if (debugEnabled == true) ROS_INFO("EXIT ITEM2");
                }
            }
            break;

            case StateClass::USER:
            {
                if (debugEnabled == true) ROS_INFO("ENTER USER");
                std::vector<geometry_msgs::Pose> poseVec;               
                poseVec.push_back(goToUser);

                planAndMove(poseVec, group, state);
                if (state == StateClass::ERROR) break;

                prevState = StateClass::USER;
                if (debugEnabled == true) ROS_INFO("EXIT USER");
            }
            break;

            case StateClass::ERROR:
            {
                error_pub.publish(err);
                ros::spinOnce();
                ros::Duration(2.0).sleep();
                ROS_INFO("Error state, starting recovery process, please wait");
                std::vector<geometry_msgs::Pose> poseVec;               
                poseVec.push_back(home);

                planAndMove(poseVec, group, state);
                if (state == StateClass::ERROR) break;             

                prevState = StateClass::ERROR;
            }
            break;
        }
    }

    ros::shutdown();
    return 0;
}