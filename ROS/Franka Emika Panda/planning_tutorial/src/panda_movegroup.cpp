
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

enum class StateClass

{

    WAITING_ORDER,
    ITEM1,
    ITEM2,
    USER,
    ERROR,
    THROW
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_movegroup");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();




    /*///////////////////////////////////////////////////////////////
     -                                                               -
     -                ////  Code for robobar Table ////              -
     -                                                               -
     ///////////////////////////////////////////////////////////////*/



    ROS_INFO("This is Robobar_Code");



    /* This sleep is ONLY to allow Rviz to come up */

    sleep(1.0);




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

    group.setGoalTolerance(0.01);

    // start kinova Gripper action client server

    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> kinova_client("/m1n6s300_driver/fingers_action/finger_positions", true);


    // wait for the Move action client server to start

    ros::WallDuration(1.0).sleep();

    ROS_INFO("Action server started");


    // set the fingers to open position

    kinova_msgs::SetFingersPositionGoal open;

    open.fingers.finger1 = 0.0;
    open.fingers.finger2 = 0.0;
    open.fingers.finger3 = 0.0;

    // set the fingers to close position

    kinova_msgs::SetFingersPositionGoal close;

    close.fingers.finger1 = 6000.0;
    close.fingers.finger2 = 6000.0;
    close.fingers.finger3 = 6000.0;


    // Initiate The Error Recovery Action for Franka

    ros::Publisher error_pub = node_handle.advertise<franka_control::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 1000);

    ros::Rate loop_rate(50);

    // Publish an Empty Messeges For Error Recovery

    franka_control::ErrorRecoveryActionGoal err;


    StateClass state = StateClass::WAITING_ORDER;

    while (ros::ok())

    {

        switch (state)

        {
            case StateClass::WAITING_ORDER:

            {
                ROS_INFO("ENTER WAITING_ORDER");  
                std::vector<geometry_msgs::Pose> def_again;

                geometry_msgs::Pose target_def_again;

                target_def_again.position.x = 0.963962;
                target_def_again.position.y = -0.0894077;
                target_def_again.position.z = 0.65493;
                target_def_again.orientation.x = -0.033849;
                target_def_again.orientation.y = -0.009381;
                target_def_again.orientation.z = 0.309561;
                target_def_again.orientation.w = 0.950232;


                def_again.push_back(target_def_again); // default again position



                moveit_msgs::RobotTrajectory trajectory_7;
                double fraction_7 = group.computeCartesianPath(def_again,
                                                               0.01,  // eef_step
                                                               0.0,   // jump_threshold
                                                               trajectory_7);

                moveit::planning_interface::MoveGroup::Plan def_again_plan;
                def_again_plan.trajectory_ = trajectory_7;



                // Excute cartesian Path And Also Check For Errors

                if (!group.execute(def_again_plan))

                {
                    ROS_INFO("Default Again Plan Failed, changing the state to ERROR");

                    state = StateClass::ERROR;

                    break;
                }

                ROS_INFO("Please Press Enter To Start Serving");    // add cntr+c to END

                char pause = 'x';
                ROS_INFO("Please Press b for bottle, t for tea");    // add cntr+c to END
                do 
                {   
                    pause = getchar();
                    if (pause == 'b') {
                        state = StateClass::ITEM1;
                        kinova_client.sendGoal(open);
                        break;
                    }
                    else if(pause == 't') {
                        state = StateClass::ITEM2;
                        kinova_client.sendGoal(open);
                        break;
                    }
                    ros::Duration(0.1).sleep();                 
                } while (true);
                ROS_INFO("EXIT WAITING_ORDER");  
                

            }

                break;

            case StateClass::ITEM1:

            {

                ROS_INFO("ENTER ITEM1");
                group.clearPathConstraints();
                // Set Kinova Gripper to Open Position

                //kinova_client.sendGoal(open);

                while(!kinova_client.getState().isDone())
                {

                    // just looping until it finishes the required action
                    ros::Duration(0.1).sleep();

                }

                std::vector<geometry_msgs::Pose> grasp;

                geometry_msgs::Pose grasp_cup;

                grasp_cup.position.x = 1.25399;
                grasp_cup.position.y = -0.266322;
                grasp_cup.position.z = 0.276048;
                grasp_cup.orientation.x = 0.689556;
                grasp_cup.orientation.y = 0.199959;
                grasp_cup.orientation.z = -0.628889;
                grasp_cup.orientation.w = -0.298375;


                grasp.push_back(grasp_cup); 

                grasp_cup.position.x = 1.3364;
                grasp_cup.position.y = -0.2755;
                grasp_cup.position.z = 0.2906;
                grasp_cup.orientation.x = 0.686;
                grasp_cup.orientation.y = 0.198;
                grasp_cup.orientation.z = -0.6321;
                grasp_cup.orientation.w = -0.3009;


                grasp.push_back(grasp_cup); 

                moveit_msgs::RobotTrajectory trajectory_g;
                double fraction_g = group.computeCartesianPath(grasp,
                                                               0.01,  // eef_step
                                                               0.0,   // jump_threshold
                                                               trajectory_g);

                moveit::planning_interface::MoveGroup::Plan grasp_plan;
                grasp_plan.trajectory_ = trajectory_g;



                // Excute cartesian Path And Also Check For Errors


                if (!group.execute(grasp_plan))

                {
                    ROS_INFO("Grasp Plan Failed, changing the state to ERROR State");

                    state = StateClass::ERROR;

                    break;
                }
                ROS_INFO("Grasp Plan Completed Successfully");

               /* char pause = 'x';
                ROS_INFO("Please Press 1 To continue");    // add cntr+c to END
                do 
                {   
                    pause = getchar();
                    ros::Duration(0.1).sleep();                 
                } while (pause != '1');
                pause = 'x';*/
                kinova_client.sendGoal(close);
                while(!kinova_client.getState().isDone())

                {

                    // just looping until it finishes the required action

                    ros::Duration(0.1).sleep();

                }
                /*ROS_INFO("Please Press 2 To continue");    // add cntr+c to END
                do 
                {
                    pause = getchar();
                    ros::Duration(0.1).sleep();                 
                } while (pause != '2');
*/
                state = StateClass::USER;
                ROS_INFO("EXIT ITEM1");
            }
            break;

            case StateClass::ITEM2:

            {

                ROS_INFO("ENTER ITEM2");                
                group.clearPathConstraints();
                // Set Kinova Gripper to Open Position

                //kinova_client.sendGoal(open);

                while(!kinova_client.getState().isDone())
                {

                    // just looping until it finishes the required action
                    ros::Duration(0.1).sleep();

                }

                std::vector<geometry_msgs::Pose> grasp;

                geometry_msgs::Pose grasp_cup;

                grasp_cup.position.x = 1.3182;
                grasp_cup.position.y = -0.0041;
                grasp_cup.position.z = 0.53;
                grasp_cup.orientation.x = -0.0283;
                grasp_cup.orientation.y = 0.0302;
                grasp_cup.orientation.z = 0.8897;
                grasp_cup.orientation.w = 0.4545;


                grasp.push_back(grasp_cup); 

                grasp_cup.position.x = 1.3251;
                grasp_cup.position.y = -0.0008;
                grasp_cup.position.z = 0.4099;
                grasp_cup.orientation.x = -0.0045;
                grasp_cup.orientation.y = 0.0093;
                grasp_cup.orientation.z = 0.9261;
                grasp_cup.orientation.w = 0.3771;


                grasp.push_back(grasp_cup); 

                moveit_msgs::RobotTrajectory trajectory_g;
                double fraction_g = group.computeCartesianPath(grasp,
                                                               0.01,  // eef_step
                                                               0.0,   // jump_threshold
                                                               trajectory_g);

                moveit::planning_interface::MoveGroup::Plan grasp_plan;
                grasp_plan.trajectory_ = trajectory_g;



                // Excute cartesian Path And Also Check For Errors


                if (!group.execute(grasp_plan))

                {
                    ROS_INFO("Grasp Plan Failed, changing the state to ERROR State");

                    state = StateClass::ERROR;

                    break;
                }
                ROS_INFO("Grasp Plan Completed Successfully");

                /*char pause = 'x';
                ROS_INFO("Please Press 1 To continue");    // add cntr+c to END
                do 
                {   
                    pause = getchar();
                    ros::Duration(0.1).sleep();                 
                } while (pause != '1');
                pause = 'x';
                */
                kinova_client.sendGoal(close);
                while(!kinova_client.getState().isDone())

                {

                    // just looping until it finishes the required action

                    ros::Duration(0.1).sleep();

                }
                ROS_INFO("Please Press 2 To continue");    // add cntr+c to END
                /*do 
                {
                    pause = getchar();
                    ros::Duration(0.1).sleep();                 
                } while (pause != '2');
*/
                state = StateClass::USER;
                ROS_INFO("EXIT ITEM2");
            }
            break;

            case StateClass::USER:
            {
                ROS_INFO("ENTER USER");
                std::vector<geometry_msgs::Pose> bottle;

                geometry_msgs::Pose target_pose_up;


                target_pose_up.position.x = 0.9818;
                target_pose_up.position.y = 0.2082;
                target_pose_up.position.z = 0.6347;
                //target_pose_up.orientation.x = 0.515;
                //target_pose_up.orientation.y = -0.351;
                //target_pose_up.orientation.z = 0.185;
                //target_pose_up.orientation.w = 0.759;
                target_pose_up.orientation.x = -0.3823;
                target_pose_up.orientation.y = -0.5232;
                target_pose_up.orientation.z = 0.7516;
                target_pose_up.orientation.w = -0.1231;
                bottle.push_back(target_pose_up);  // go up to the bottle



                moveit_msgs::RobotTrajectory trajectory_3;
                double fraction_3 = group.computeCartesianPath(bottle,
                                                               0.01,  // eef_step
                                                               0.0,   // jump_threshold
                                                               trajectory_3);

                moveit::planning_interface::MoveGroup::Plan bottle_plan;
                bottle_plan.trajectory_ = trajectory_3;




                if (!group.execute(bottle_plan))

                {
                    ROS_INFO("Bottle Plan Failed, changing the state to THROW State");

                    state = StateClass::THROW;

                    break;
                }
                
/*
                group.setNamedTarget("user");
                if (!group.move())

                {
                    ROS_INFO("Grasp Plan Failed, changing the state to ERROR State");

                    state = StateClass::ERROR;

                    break;
                }
                */
                ros::Duration(0.3).sleep();

             
                char pause = 'x';
                ROS_INFO("Please Press 3 To continue");    // add cntr+c to END
                do 
                {   
                    pause = getchar();
                    ros::Duration(0.1).sleep();                 
                } while (pause != '3');
                /*
                group.setNamedTarget("item1");
                if (!group.move())

                {
                    ROS_INFO("Grasp Plan Failed, changing the state to ERROR State");

                    state = StateClass::ERROR;

                    break;
                }
                group.setNamedTarget("user");
                if (!group.move())

                {
                    ROS_INFO("Grasp Plan Failed, changing the state to ERROR State");

                    state = StateClass::ERROR;

                    break;
                }
                */

                state = StateClass::WAITING_ORDER;
                ROS_INFO("EXIT USER");
            }

                break;

            case StateClass::THROW:

            {

                error_pub.publish(err);

                ros::spinOnce();

                ros::Duration(0.2).sleep();

                ROS_INFO("THROW State Started, Please wait");

                std::vector<geometry_msgs::Pose> throw_out;

                geometry_msgs::Pose throw_target;

                throw_target.position.x = 1.29021;
                throw_target.position.y = -0.07857;
                throw_target.position.z = 0.25404;
                throw_target.orientation.x = 0.685891;
                throw_target.orientation.y = 0.221853;
                throw_target.orientation.z = -0.63797;
                throw_target.orientation.w = -0.270778;


                throw_out.push_back(throw_target); // Throw position


                moveit_msgs::RobotTrajectory trajectory_T;
                double fraction_T = group.computeCartesianPath(throw_out,
                                                               0.01,  // eef_step
                                                               0.0,   // jump_threshold
                                                               trajectory_T);

                moveit::planning_interface::MoveGroup::Plan throw_out_plan;
                throw_out_plan.trajectory_ = trajectory_T;


                // Excute cartesian Path And Also Check For Errors

                if (!group.execute(throw_out_plan))

                {
                    ROS_INFO("Throw Out Plan Failed, Repeating the THROW State Again");

                    state = StateClass::THROW;

                    break;
                }

                ros::Duration(0.2).sleep();

                kinova_client.sendGoal(open);

                ROS_INFO("Throw Out Plan Completed Successfully");


                while(!kinova_client.getState().isDone())

                {

                    // just looping until it finishes the required action

                    ros::Duration(0.1).sleep();

                }

                std::vector<geometry_msgs::Pose> throw_def;

                geometry_msgs::Pose target_throw_def;

                target_throw_def.position.x = 0.963962;
                target_throw_def.position.y = -0.0894077;
                target_throw_def.position.z = 0.65493;
                target_throw_def.orientation.x = -0.033849;
                target_throw_def.orientation.y = -0.009381;
                target_throw_def.orientation.z = 0.309561;
                target_throw_def.orientation.w = 0.950232;


                throw_def.push_back(target_throw_def); // default position



                moveit_msgs::RobotTrajectory trajectory_TF;
                double fraction_TF = group.computeCartesianPath(throw_def,
                                                                0.01,  // eef_step
                                                                0.0,   // jump_threshold
                                                                trajectory_TF);

                moveit::planning_interface::MoveGroup::Plan throw_def_plan;
                throw_def_plan.trajectory_ = trajectory_TF;



                // Excute cartesian Path And Also Check For Errors

                if (!group.execute(throw_def_plan))

                {
                    ROS_INFO("Default Plan Failed, changing the state to ERROR");

                    state = StateClass::ERROR;

                    break;
                }

                state = StateClass::WAITING_ORDER;


            }

                break;


            case StateClass::ERROR:

            {
                error_pub.publish(err);

                ros::spinOnce();

                ros::Duration(3.5).sleep();

                ROS_INFO("Error State , Starting Recovery Process, Please wait");

                std::vector<geometry_msgs::Pose> def;

                geometry_msgs::Pose target_def;

                target_def.position.x = 0.963962;
                target_def.position.y = -0.0894077;
                target_def.position.z = 0.65493;
                target_def.orientation.x = -0.033849;
                target_def.orientation.y = -0.009381;
                target_def.orientation.z = 0.309561;
                target_def.orientation.w = 0.950232;

                def.push_back(target_def); // default position


                moveit_msgs::RobotTrajectory trajectory_1;
                double fraction_1 = group.computeCartesianPath(def,
                                                               0.01,  // eef_step
                                                               0.0,   // jump_threshold
                                                               trajectory_1);

                moveit::planning_interface::MoveGroup::Plan def_plan;
                def_plan.trajectory_ = trajectory_1;

                // Excute cartesian Path And Also Check For Errors

                if (!group.execute(def_plan))

                {
                    ROS_INFO("Default Plan Failed, changing the state to ERROR");

                    state = StateClass::ERROR;

                    break;
                }


                state = StateClass::WAITING_ORDER;

            }
                break;

        }


    }






    /*///////////////////////////////////////////////////////////////
     -                                                               -
     -                ////  Code for robobar ////                    -
     -                                                               -
     ///////////////////////////////////////////////////////////////*/



    //   ROS_INFO("This is Robobar_Code");



    //   /* This sleep is ONLY to allow Rviz to come up */

    //   sleep(5.0);




    //   // Setup
    //   // ^^^^^
    //   //
    //   // The :move_group_interface:`MoveGroup` class can be easily
    //   // setup using just the name
    //   // of the group you would like to control and plan for.

    //   moveit::planning_interface::MoveGroup group("panda_arm_hand");


    //   // We will use the :planning_scene_interface:`PlanningSceneInterface`
    //   // class to deal directly with the world.
    //   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //   // (Optional) Create a publisher for visualizing plans in Rviz.
    //   ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //   moveit_msgs::DisplayTrajectory display_trajectory;



    //   // Set Franka external force Threshold

    //   ros::ServiceClient client = node_handle.serviceClient<franka_control::SetFullCollisionBehavior>("/panda/set_full_collision_behavior");

    //   franka_control::SetFullCollisionBehavior srv;


    //   srv.request.lower_force_thresholds_acceleration.assign (100.0);
    //   srv.request.lower_force_thresholds_nominal.assign (100.0);
    //   srv.request.lower_torque_thresholds_acceleration.assign (100.0);
    //   srv.request.lower_torque_thresholds_nominal.assign (100.0);

    //   srv.request.upper_force_thresholds_acceleration.assign (100.0);
    //   srv.request.upper_force_thresholds_nominal.assign (100.0);
    //   srv.request.upper_torque_thresholds_acceleration.assign (100.0);
    //   srv.request.upper_torque_thresholds_nominal.assign (100.0);

    //   client.call(srv);


    ////   // Initiate The Error Recovery Action for Franka

    ////   ros::Publisher error_pub = node_handle.advertise<franka_control::ErrorRecoveryActionGoal>("/panda/error_recovery/goal", 1000);

    ////   ros::Rate loop_rate(10);

    ////    while (ros::ok())

    ////    {

    ////        // Publish an Empty Messeges

    ////        franka_control::ErrorRecoveryActionGoal err;


    ////        error_pub.publish(err);

    ////        ros::spinOnce();

    ////        loop_rate.sleep();
    ////    }



    //   // start kinova Gripper action client server

    //   actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> kinova_client("/m1n6s300_driver/fingers_action/finger_positions", true);


    //   // wait for the Move action client server to start

    //   ros::WallDuration(1.0).sleep();

    //   ROS_INFO("Action server started");


    //   // set the fingers to open position

    //   kinova_msgs::SetFingersPositionGoal open;

    //   open.fingers.finger1 = 0.0;
    //   open.fingers.finger2 = 0.0;
    //   open.fingers.finger3 = 0.0;

    //   kinova_client.sendGoal(open);

    //    while(!kinova_client.getState().isDone())
    //    {

    //    // just looping until it finishes the required action
    //    ros::Duration(0.1).sleep();

    //    }


    //    std::vector<geometry_msgs::Pose> def;

    //    geometry_msgs::Pose target_def;

    //    target_def.position.x = 0.963962;
    //    target_def.position.y = -0.0894077;
    //    target_def.position.z = 0.65493;
    //    target_def.orientation.x = -0.033849;
    //    target_def.orientation.y = -0.009381;
    //    target_def.orientation.z = 0.309561;
    //    target_def.orientation.w = 0.950232;


    //    def.push_back(target_def); // default position



    //    moveit_msgs::RobotTrajectory trajectory_1;
    //    double fraction_1 = group.computeCartesianPath(def,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory_1);

    //    moveit::planning_interface::MoveGroup::Plan def_plan;
    //    def_plan.trajectory_ = trajectory_1;


    //    ROS_INFO("Visualizing cartesian path one (%.2f%% acheived)",
    //        fraction_1 * 100.0);

    //    // Excute first cartesian path

    //     group.execute(def_plan);


    //    std::vector<geometry_msgs::Pose> grasp;

    //    geometry_msgs::Pose grasp_cup;

    //    grasp_cup.position.x = 1.25399;
    //    grasp_cup.position.y = -0.266322;
    //    grasp_cup.position.z = 0.276048;
    //    grasp_cup.orientation.x = 0.689556;
    //    grasp_cup.orientation.y = 0.199959;
    //    grasp_cup.orientation.z = -0.628889;
    //    grasp_cup.orientation.w = -0.298375;


    //    grasp.push_back(grasp_cup); // grasp the cup



    //    moveit_msgs::RobotTrajectory trajectory_g;
    //    double fraction_g = group.computeCartesianPath(grasp,
    //                                            0.01,  // eef_step
    //                                            0.0,   // jump_threshold
    //                                            trajectory_g);

    //    moveit::planning_interface::MoveGroup::Plan grasp_plan;
    //    grasp_plan.trajectory_ = trajectory_g;


    //    ROS_INFO("Visualizing cartesian path one (%.2f%% acheived)",
    //     fraction_g * 100.0);

    //    // Excute first cartesian path

    //    group.execute(grasp_plan);




    //   ros::Duration(0.5).sleep();

    //   kinova_msgs::SetFingersPositionGoal close;

    //   close.fingers.finger1 = 6400.0;
    //   close.fingers.finger2 = 6400.0;
    //   close.fingers.finger3 = 6400.0;

    //   kinova_client.sendGoal(close);

    //    while(!kinova_client.getState().isDone())

    //    {

    //    // just looping until it finishes the required action

    //    ros::Duration(0.1).sleep();

    //    }



    //   std::vector<geometry_msgs::Pose> bottle;

    //   geometry_msgs::Pose target_pose_up;


    //   target_pose_up.position.x = 0.946362;
    //   target_pose_up.position.y = -0.090462;
    //   target_pose_up.position.z = 0.934091;
    //   target_pose_up.orientation.x = 0.718577;
    //   target_pose_up.orientation.y = 0.192931;
    //   target_pose_up.orientation.z = -0.597533;
    //   target_pose_up.orientation.w = -0.298961;

    //   bottle.push_back(target_pose_up);  // go up to the bottle



    //   moveit_msgs::RobotTrajectory trajectory_3;
    //   double fraction_3 = group.computeCartesianPath(bottle,
    //                                              0.01,  // eef_step
    //                                              0.0,   // jump_threshold
    //                                              trajectory_3);

    //   moveit::planning_interface::MoveGroup::Plan bottle_plan;
    //   bottle_plan.trajectory_ = trajectory_3;

    //   ROS_INFO("Visualizing cartesian path two (%.2f%% acheived)",
    //       fraction_3 * 100.0);


    //   // Excute Second cartesian path

    //    group.execute(bottle_plan);


    //    ros::Duration(0.3).sleep();


    //    std::vector<geometry_msgs::Pose> bottle_fill;

    //    geometry_msgs::Pose target_pose_fill;


    //    target_pose_fill.position.x = 0.946362;
    //    target_pose_fill.position.y = -0.090462;
    //    target_pose_fill.position.z = 0.980091;
    //    target_pose_fill.orientation.x = 0.718577;
    //    target_pose_fill.orientation.y = 0.192931;
    //    target_pose_fill.orientation.z = -0.597533;
    //    target_pose_fill.orientation.w = -0.298961;

    //    bottle_fill.push_back(target_pose_fill);  // fill the glass


    //    moveit_msgs::RobotTrajectory trajectory_4;
    //    double fraction_4 = group.computeCartesianPath(bottle_fill,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory_4);

    //    moveit::planning_interface::MoveGroup::Plan bottle_fill_plan;
    //    bottle_fill_plan.trajectory_ = trajectory_4;

    //    ROS_INFO("Visualizing cartesian path two (%.2f%% acheived)",
    //        fraction_4 * 100.0);


    //   // Excute Second cartesian path

    //    group.execute(bottle_fill_plan);


    //    ros::Duration(2.0).sleep();


    //    std::vector<geometry_msgs::Pose> finish;

    //    geometry_msgs::Pose target_finish;

    //    target_finish.position.x = 0.946362;
    //    target_finish.position.y = -0.090462;
    //    target_finish.position.z = 0.934091;
    //    target_finish.orientation.x = 0.718577;
    //    target_finish.orientation.y = 0.192931;
    //    target_finish.orientation.z = -0.597533;
    //    target_finish.orientation.w = -0.298961;

    //    finish.push_back(target_finish);  // go down little



    //    moveit_msgs::RobotTrajectory trajectory_5;
    //    double fraction_5 = group.computeCartesianPath(finish,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory_5);

    //    moveit::planning_interface::MoveGroup::Plan finish_plan;
    //    finish_plan.trajectory_ = trajectory_5;


    //    ROS_INFO("Visualizing cartesian path one (%.2f%% acheived)",
    //        fraction_5 * 100.0);

    //    // Excute cartesian path


    //     group.execute(finish_plan);


    //     // Define the path constraints

    //     // ROS uses Radians unit for angles, so convert from degrees to radians by : rad = degree * 3.14159 / 180


    //     moveit_msgs::OrientationConstraint ocm;
    //     ocm.link_name = "m1n6s300_link_6";
    //     ocm.header.frame_id = "/world";
    //     ocm.orientation.w = 1.0;
    //     ocm.absolute_x_axis_tolerance = 0.5;
    //     ocm.absolute_y_axis_tolerance = 0.1;
    //     ocm.absolute_z_axis_tolerance = 0.1;
    //     ocm.weight = 1.0;

    //     // Now, set it as the path constraint for the group.

    //     moveit_msgs::Constraints constraint_1;
    //     constraint_1.orientation_constraints.push_back(ocm);


    //     group.setPathConstraints(constraint_1);


    //     std::vector<geometry_msgs::Pose> place;

    //     geometry_msgs::Pose target_place;


    //     target_place.position.x = 1.25399;
    //     target_place.position.y = -0.266322;
    //     target_place.position.z = 0.276048;
    //     target_place.orientation.x = 0.689556;
    //     target_place.orientation.y = 0.199959;
    //     target_place.orientation.z = -0.628889;
    //     target_place.orientation.w = -0.298375;


    //     place.push_back(target_place);  // put the cup back



    //     moveit_msgs::RobotTrajectory trajectory_6;
    //     double fraction_6 = group.computeCartesianPath(place,
    //                                                0.01,  // eef_step
    //                                                0.0,   // jump_threshold
    //                                                trajectory_6);

    //     moveit::planning_interface::MoveGroup::Plan place_plan;
    //     place_plan.trajectory_ = trajectory_6;


    //     ROS_INFO("Visualizing cartesian path one (%.2f%% acheived)",
    //         fraction_6 * 100.0);


    //     // Excute cartesian path


    //      group.execute(place_plan);

    //      ros::Duration(0.9).sleep();


    //    // Keep the gripper closed to avoid drop down the cup in case of Franka error

    //    if (group.execute(place_plan).FAILURE)
    //    {

    //    kinova_client.sendGoal(close);

    //    }


    //    else if (group.execute(place_plan).SUCCESS)
    //    {

    //    kinova_client.sendGoal(open);

    //    }


    //    while(!kinova_client.getState().isDone())
    //    {

    //    // just looping until it finishes the required action
    //    ros::Duration(0.1).sleep();

    //    }

    //    // When done with the path constraint be sure to clear it.
    //    group.clearPathConstraints();


    //    std::vector<geometry_msgs::Pose> def_again;

    //    geometry_msgs::Pose target_def_again;

    //    target_def_again.position.x = 0.963962;
    //    target_def_again.position.y = -0.0894077;
    //    target_def_again.position.z = 0.65493;
    //    target_def_again.orientation.x = -0.033849;
    //    target_def_again.orientation.y = -0.009381;
    //    target_def_again.orientation.z = 0.309561;
    //    target_def_again.orientation.w = 0.950232;


    //    def_again.push_back(target_def_again); // def_againault position



    //    moveit_msgs::RobotTrajectory trajectory_7;
    //    double fraction_7 = group.computeCartesianPath(def_again,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory_7);

    //    moveit::planning_interface::MoveGroup::Plan def_again_plan;
    //    def_again_plan.trajectory_ = trajectory_7;


    //    ROS_INFO("Visualizing cartesian path one (%.2f%% acheived)",
    //        fraction_7 * 100.0);

    //    // Excute first cartesian path

    //     group.execute(def_again_plan);



    //   /*///////////////////////////////////////////////////////////////
    //   -                                                               -
    //   -                ////  Code for Franka-Kinova ////              -
    //   -                                                               -
    //   ///////////////////////////////////////////////////////////////*/



    //    ROS_INFO("This is Franka_Kinova_Code");



    //    /* This sleep is ONLY to allow Rviz to come up */

    //    sleep(5.0);



    //    // Setup
    //    // ^^^^^
    //    //
    //    // The :move_group_interface:`MoveGroup` class can be easily
    //    // setup using just the name
    //    // of the group you would like to control and plan for.

    //    moveit::planning_interface::MoveGroup group("panda_arm_hand");


    //    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    //    // class to deal directly with the world.
    //    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //    // (Optional) Create a publisher for visualizing plans in Rviz.
    //    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //    moveit_msgs::DisplayTrajectory display_trajectory;



    //    // start kinova action client server

    //    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> kinova_client("/m1n6s300_driver/fingers_action/finger_positions", true);


    //    // wait for the Move action client server to start

    //    ros::WallDuration(2.0).sleep();

    //    ROS_INFO("Action server started");


    //    // set the fingers to open position

    //    kinova_msgs::SetFingersPositionGoal open;

    //    open.fingers.finger1 = 0.0;
    //    open.fingers.finger2 = 0.0;
    //    open.fingers.finger3 = 0.0;

    //    kinova_client.sendGoal(open);


    //    std::vector<geometry_msgs::Pose> grasp;

    //    geometry_msgs::Pose target_pose_1;

    //    target_pose_1.position.x = 0.90196;
    //    target_pose_1.position.y = -0.023556;
    //    target_pose_1.position.z = 0.672764;
    //    target_pose_1.orientation.x = 0.034101;
    //    target_pose_1.orientation.y = -0.0199791;
    //    target_pose_1.orientation.z = 0.304773;
    //    target_pose_1.orientation.w = 0.951604;


    //    grasp.push_back(target_pose_1); // default position


    //    target_pose_1.position.x = 0.92892;
    //    target_pose_1.position.y = 0.15367;
    //    target_pose_1.position.z = 0.32031;
    //    target_pose_1.orientation.x = 0.015608;
    //    target_pose_1.orientation.y = -0.00400;
    //    target_pose_1.orientation.z = 0.307639;
    //    target_pose_1.orientation.w = 0.951366;


    //    grasp.push_back(target_pose_1);  // Point A


    //    // We want the cartesian path to be interpolated at a resolution of 1 cm
    //    // which is why we will specify 0.01 as the max step in cartesian
    //    // translation.  We will specify the jump threshold as 0.0, effectively
    //    // disabling it.


    //    moveit_msgs::RobotTrajectory trajectory_1;
    //    double fraction_1 = group.computeCartesianPath(grasp,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory_1);

    //    moveit::planning_interface::MoveGroup::Plan grasp_plan;
    //    grasp_plan.trajectory_ = trajectory_1;


    //    ROS_INFO("Visualizing cartesian path one (%.2f%% acheived)",
    //        fraction_1 * 100.0);

    //    // Excute first cartesian path

    //     group.execute(grasp_plan);


    //    ros::Duration(0.1).sleep();

    //    // set the fingers to grasp position

    //    kinova_msgs::SetFingersPositionGoal close;

    //    close.fingers.finger1 = 6400.0;
    //    close.fingers.finger2 = 6400.0;
    //    close.fingers.finger3 = 6400.0;

    //    kinova_client.sendGoal(close);



    //    while(!kinova_client.getState().isDone())
    //    {

    //    // just looping until it finishes the required action
    //    ros::Duration(0.1).sleep();

    //    }

    //    std::vector<geometry_msgs::Pose> release;

    //    geometry_msgs::Pose target_pose_2;

    //    target_pose_2.position.x = 1.1986;
    //    target_pose_2.position.y = 0.003956;
    //    target_pose_2.position.z = 0.44126;
    //    target_pose_2.orientation.x = 0.0217006;
    //    target_pose_2.orientation.y = -0.000956;
    //    target_pose_2.orientation.z = 0.28334;
    //    target_pose_2.orientation.w = 0.958775;

    //    release.push_back(target_pose_2);  // Point B

    //    target_pose_2.position.x = 1.42744;
    //    target_pose_2.position.y = -0.0130651;
    //    target_pose_2.position.z = 0.302087;
    //    target_pose_2.orientation.x = 0.007800;
    //    target_pose_2.orientation.y = -0.0102331;
    //    target_pose_2.orientation.z = 0.289835;
    //    target_pose_2.orientation.w = 0.956991;

    //    release.push_back(target_pose_2);  // Point C


    //    moveit_msgs::RobotTrajectory trajectory_2;
    //    double fraction_2 = group.computeCartesianPath(release,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory_2);

    //    moveit::planning_interface::MoveGroup::Plan release_plan;
    //    release_plan.trajectory_ = trajectory_2;

    //    ROS_INFO("Visualizing cartesian path two (%.2f%% acheived)",
    //        fraction_2 * 100.0);

    //    // Excute Second cartesian path


    //     group.execute(release_plan);

    //    ros::Duration(0.1).sleep();

    //    kinova_client.sendGoal(open);


    //    while (!kinova_client.getState().isDone());

    //    {

    //    // just looping until it finishes the required action
    //    ros::Duration(0.1).sleep();
    //    }

    //    std::vector<geometry_msgs::Pose> end;

    //    geometry_msgs::Pose target_pose_up;

    //    target_pose_up.position.x = 1.42744;
    //    target_pose_up.position.y = -0.0130651;
    //    target_pose_up.position.z = 0.369737;
    //    target_pose_up.orientation.x = 0.007800;
    //    target_pose_up.orientation.y = -0.0102331;
    //    target_pose_up.orientation.z = 0.289835;
    //    target_pose_up.orientation.w = 0.956991;

    //    end.push_back(target_pose_up);  // go up liitle in Point C



    //    target_pose_up.position.x = 0.90196;
    //    target_pose_up.position.y = -0.023556;
    //    target_pose_up.position.z = 0.672764;
    //    target_pose_up.orientation.x = 0.034101;
    //    target_pose_up.orientation.y = -0.0199791;
    //    target_pose_up.orientation.z = 0.304773;
    //    target_pose_up.orientation.w = 0.951604;


    //    end.push_back(target_pose_up);  // Go Back To The Default Position


    //    moveit_msgs::RobotTrajectory trajectory_3;
    //    double fraction_3 = group.computeCartesianPath(end,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory_3);

    //    moveit::planning_interface::MoveGroup::Plan end_plan;
    //    end_plan.trajectory_ = trajectory_3;

    //    ROS_INFO("Visualizing cartesian path two (%.2f%% acheived)",
    //        fraction_3 * 100.0);

    //    // Excute Second cartesian path

    //     group.execute(end_plan);




    ///////////////////////////////////////////////////////////////////
    //-                                                               -
    //-                //// Code for Franka-Generated ////            -
    //-                                                               -
    ///////////////////////////////////////////////////////////////////


    //  ROS_INFO("This is Franka_Generated_Code");



    //  /* This sleep is ONLY to allow Rviz to come up */

    //  sleep(5.0);




    //  // Setup
    //  // ^^^^^
    //  //
    //  // The :move_group_interface:`MoveGroup` class can be easily
    //  // setup using just the name
    //  // of the group you would like to control and plan for.
    //  moveit::planning_interface::MoveGroup group("arm_hand");


    //  // We will use the :planning_scene_interface:`PlanningSceneInterface`
    //  // class to deal directly with the world.
    //  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //  // (Optional) Create a publisher for visualizing plans in Rviz.
    //  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //  moveit_msgs::DisplayTrajectory display_trajectory;




    //  // Getting Basic Information
    //    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //    //
    //    // We can print the name of the reference frame for this robot.
    //    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    //    // We can also print the name of the end-effector link for this group.
    //    ROS_INFO("End-effector link: %s", group.getEndEffectorLink().c_str());



    //    // Cartesian different Paths


    //    //  // We will reuse the old goal that we had and plan to it.
    //    //  // Note that this will only work if the current state already
    //    //  // satisfies the path constraints. So, we need to set the start
    //    //  // state to a new pose.




    //    // set the grippers to the default position

    //    actionlib::SimpleActionClient<franka_gripper::MoveAction> ac("franka_gripper_node/move",true);


    //    // wait for the Move action client server to start

    //    ac.waitForServer(); //will wait for infinite time

    //    ROS_INFO("Action server started");


    //    // send a goal to the action

    //    franka_gripper::MoveGoal def;

    //    def.speed = 0.1;
    //    def.width = 0.08;

    //    ac.sendGoal(def);




    //    std::vector<geometry_msgs::Pose> grasp;

    //    geometry_msgs::Pose target_pose_1;

    //    target_pose_1.position.x = 0.892240;
    //    target_pose_1.position.y = 0.003596;
    //    target_pose_1.position.z = 0.645421;
    //    target_pose_1.orientation.x = 0.75841;
    //    target_pose_1.orientation.y = 0.651183;
    //    target_pose_1.orientation.z = 0.021872;
    //    target_pose_1.orientation.w = -0.017133;


    //    grasp.push_back(target_pose_1); // default position


    //    target_pose_1.position.x = 0.921482;
    //    target_pose_1.position.y = 0.102755;
    //    target_pose_1.position.z = 0.196277;
    //    target_pose_1.orientation.x = 0.76533;
    //    target_pose_1.orientation.y = 0.64338;
    //    target_pose_1.orientation.z = 0.0091018;
    //    target_pose_1.orientation.w = -0.017430;


    //    grasp.push_back(target_pose_1);  // Point A


    //    // We want the cartesian path to be interpolated at a resolution of 1 cm
    //    // which is why we will specify 0.01 as the max step in cartesian
    //    // translation.  We will specify the jump threshold as 0.0, effectively
    //    // disabling it.



    //    moveit_msgs::RobotTrajectory trajectory_1;
    //    double fraction_1 = group.computeCartesianPath(grasp,
    //                                                 0.01,  // eef_step
    //                                                 0.0,   // jump_threshold
    //                                                 trajectory_1);

    //    moveit::planning_interface::MoveGroup::Plan grasp_plan;
    //    grasp_plan.trajectory_ = trajectory_1;


    //    ROS_INFO("Visualizing cartesian path one (%.2f%% acheived)",
    //          fraction_1 * 100.0);

    //    // Excute first cartesian path

    //    group.execute(grasp_plan);


    //    ros::Duration(0.1).sleep();


    //    actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_1("franka_gripper_node/grasp",true);

    //    ac_1.waitForServer(); //will wait for infinite time

    //    ROS_INFO("Action server started");


    //    franka_gripper::GraspGoal gripper_grasp;

    //    gripper_grasp.speed = 0.2;
    //    gripper_grasp.width = 0.05;
    //    gripper_grasp.force = 0.1;

    //    ac_1.sendGoal(gripper_grasp);

    //    while(!ac_1.getState().isDone())
    //    {
    //      // just looping
    //      ros::Duration(0.1).sleep();

    //    }

    //    std::vector<geometry_msgs::Pose> release;

    //    geometry_msgs::Pose target_pose_2;

    //    target_pose_2.position.x = 1.20223;
    //    target_pose_2.position.y = -0.01695;
    //    target_pose_2.position.z = 0.488511;
    //    target_pose_2.orientation.x = 0.770135;
    //    target_pose_2.orientation.y = 0.637445;
    //    target_pose_2.orientation.z = 0.008117;
    //    target_pose_2.orientation.w = -0.022241;

    //    release.push_back(target_pose_2);  // Point B

    //    target_pose_2.position.x = 1.43177;
    //    target_pose_2.position.y = -0.032014;
    //    target_pose_2.position.z = 0.200573;
    //    target_pose_2.orientation.x = 0.75948;
    //    target_pose_2.orientation.y = 0.648536;
    //    target_pose_2.orientation.z = 0.033341;
    //    target_pose_2.orientation.w = -0.038484;

    //    release.push_back(target_pose_2);  // Point C


    //    moveit_msgs::RobotTrajectory trajectory_2;
    //    double fraction_2 = group.computeCartesianPath(release,
    //                                                 0.01,  // eef_step
    //                                                 0.0,   // jump_threshold
    //                                                 trajectory_2);

    //    moveit::planning_interface::MoveGroup::Plan release_plan;
    //    release_plan.trajectory_ = trajectory_2;

    //    ROS_INFO("Visualizing cartesian path two (%.2f%% acheived)",
    //          fraction_2 * 100.0);

    //  // Excute Second cartesian path


    //    group.execute(release_plan);

    //    ros::Duration(0.1).sleep();

    //    ac.sendGoal(def);


    //    while (!ac.getState().isDone());

    //    {

    //        //just loop
    //        ros::Duration(0.1).sleep();
    //    }

    //    std::vector<geometry_msgs::Pose> end;

    //    geometry_msgs::Pose target_pose_up;

    //    target_pose_up.position.x = 1.43177;
    //    target_pose_up.position.y = -0.032014;
    //    target_pose_up.position.z = 0.268223;
    //    target_pose_up.orientation.x = 0.75948;
    //    target_pose_up.orientation.y = 0.648536;
    //    target_pose_up.orientation.z = 0.033341;
    //    target_pose_up.orientation.w = -0.038484;

    //    end.push_back(target_pose_up);  // go up liitle in Point C



    //    target_pose_up.position.x = 0.892240;
    //    target_pose_up.position.y = 0.003596;
    //    target_pose_up.position.z = 0.645421;
    //    target_pose_up.orientation.x = 0.75841;
    //    target_pose_up.orientation.y = 0.651183;
    //    target_pose_up.orientation.z = 0.021872;
    //    target_pose_up.orientation.w = -0.017133;


    //    end.push_back(target_pose_up);  // Go Back To The Default Position


    //    moveit_msgs::RobotTrajectory trajectory_3;
    //    double fraction_3 = group.computeCartesianPath(end,
    //                                                 0.01,  // eef_step
    //                                                 0.0,   // jump_threshold
    //                                                 trajectory_3);

    //    moveit::planning_interface::MoveGroup::Plan end_plan;
    //    end_plan.trajectory_ = trajectory_3;

    //    ROS_INFO("Visualizing cartesian path two (%.2f%% acheived)",
    //          fraction_3 * 100.0);

    //  // Excute Second cartesian path

    //    group.execute(end_plan);







    /*///////////////////////////////////////////////////////////////
     -                                                               -
     -                //// Original Code for Panda ////              -
     -                                                               -
     ///////////////////////////////////////////////////////////////*/


    //  // Setup
    //  // ^^^^^
    //  //
    //  // The :move_group_interface:`MoveGroup` class can be easily
    //  // setup using just the name
    //  // of the group you would like to control and plan for.
    //  moveit::planning_interface::MoveGroup group("panda_arm_hand");


    //  // We will use the :planning_scene_interface:`PlanningSceneInterface`
    //  // class to deal directly with the world.
    //  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //  // (Optional) Create a publisher for visualizing plans in Rviz.
    //  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //  moveit_msgs::DisplayTrajectory display_trajectory;


    //  ROS_INFO("This is Panda_Original_Code");



    ////    system("roslaunch franka_gripper franka_gripper.launch robot_ip:=192.168.1.100");




    //  // Adding/Removing Objects and Attaching/Detaching Objects
    //  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //  // First, we will define the collision object message.
    //  moveit_msgs::CollisionObject collision_object_1;
    //  collision_object_1.header.frame_id = group.getPlanningFrame();

    //  /* The id of the object is used to identify it. */
    //  collision_object_1.id = "table";

    //  /* Define a table to add to the world. */
    //  shape_msgs::SolidPrimitive primitive;
    //  primitive.type = primitive.BOX;
    //  primitive.dimensions.resize(3);
    //  primitive.dimensions[0] = 3.0;
    //  primitive.dimensions[1] = 1.5;
    //  primitive.dimensions[2] = 0.02;




    //  /* A pose for the table (specified relative to frame_id) */
    //  geometry_msgs::Pose table_pose;
    //  table_pose.orientation.x = 0;
    //  table_pose.orientation.y = 0;
    //  table_pose.orientation.z = 0;
    //  table_pose.position.x = 1.2;
    //  table_pose.position.y = 0.5;
    //  table_pose.position.z = 0.0;

    //  collision_object_1.primitives.push_back(primitive);
    //  collision_object_1.primitive_poses.push_back(table_pose);
    //  collision_object_1.operation = collision_object_1.ADD;


    //  // Second, we will define the collision object 2 message.

    //  moveit_msgs::CollisionObject collision_object_2;
    //  collision_object_2.header.frame_id = group.getPlanningFrame();

    //  /* The id of the object is used to identify it. */

    //  collision_object_2.id = "left_wall";

    //  /* Define a left wall to add to the world. */

    //  primitive.type = primitive.BOX;
    //  primitive.dimensions.resize(3);
    //  primitive.dimensions[0] = 0.02;
    //  primitive.dimensions[1] = 1.5;
    //  primitive.dimensions[2] = 1;

    //  /* A pose for the left wall (specified relative to frame_id) */

    //  geometry_msgs::Pose left_wall_pose;
    //  left_wall_pose.orientation.x = 0;
    //  left_wall_pose.orientation.y = 0;
    //  left_wall_pose.orientation.z = 0;
    //  left_wall_pose.position.x = -0.3;
    //  left_wall_pose.position.y = 0.5;
    //  left_wall_pose.position.z = 0.5;


    //  collision_object_2.primitives.push_back(primitive);
    //  collision_object_2.primitive_poses.push_back(left_wall_pose);
    //  collision_object_2.operation = collision_object_2.ADD;



    //  // Third, we will define the collision object 3 message.

    //  moveit_msgs::CollisionObject collision_object_3;
    //  collision_object_3.header.frame_id = group.getPlanningFrame();

    //  /* The id of the object is used to identify it. */

    //  collision_object_3.id = "back_wall";

    //  /* Define a back wall to add to the world. */

    //  primitive.type = primitive.BOX;
    //  primitive.dimensions.resize(3);
    //  primitive.dimensions[0] = 3;
    //  primitive.dimensions[1] = 0.02;
    //  primitive.dimensions[2] = 1;

    //  /* A pose for the back wall (specified relative to frame_id) */

    //  geometry_msgs::Pose back_wall_pose;
    //  back_wall_pose.orientation.x = 0;
    //  back_wall_pose.orientation.y = 0;
    //  back_wall_pose.orientation.z = 0;
    //  back_wall_pose.position.x = 1.2;
    //  back_wall_pose.position.y = -0.24;
    //  back_wall_pose.position.z = 0.5;


    //  collision_object_3.primitives.push_back(primitive);
    //  collision_object_3.primitive_poses.push_back(back_wall_pose);
    //  collision_object_3.operation = collision_object_3.ADD;


    //  std::vector<moveit_msgs::CollisionObject> collision_objects;
    //  collision_objects.push_back(collision_object_1);
    //  collision_objects.push_back(collision_object_2);
    //  collision_objects.push_back(collision_object_3);


    //  // Now, let's add the collision object into the world
    //  ROS_INFO("Add an object into the world");
    //  sleep(5.0);

    //  planning_scene_interface.addCollisionObjects(collision_objects);

    //  /* Sle so we have time to see the object in RViz */
    //  sleep(2.0);



    // BEGIN_TUTORIAL
    //


    //  // Getting Basic Information
    //  // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //  //
    //  // We can print the name of the reference frame for this robot.
    //  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    //  // We can also print the name of the end-effector link for this group.
    //  ROS_INFO("End-effector link: %s", group.getEndEffectorLink().c_str());




    //  // Planning to a Pose goal
    //  // ^^^^^^^^^^^^^^^^^^^^^^^
    //  // We can plan a motion for this group to a desired pose for the
    //  // end-effector.

    //  geometry_msgs::Pose target_pose1;


    //    target_pose1.position.x = 0.7;
    //    target_pose1.position.y = -0.1;
    //    target_pose1.position.z = 0.1;
    //    target_pose1.orientation.x = 0.95616;
    //    target_pose1.orientation.y = 0.29148;
    //    target_pose1.orientation.z = 0.025624;
    //    target_pose1.orientation.w = -0.011784;

    //  group.setPoseTarget(target_pose1);

    ////   Now, we call the planner to compute the plan
    ////   and visualize it.
    ////   Note that we are just planning, not asking move_group
    ////   to actually move the robot.

    //  moveit::planning_interface::MoveGroup::Plan my_plan;
    //  auto success = group.plan(my_plan);

    //  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    //  /* Sleep to give Rviz time to visualize the plan. */
    //  sleep(5.0);



    /* Uncomment below line when working with a real robot*/

    //   group.move();



    // Cartesian different Paths


    //  // We will reuse the old goal that we had and plan to it.
    //  // Note that this will only work if the current state already
    //  // satisfies the path constraints. So, we need to set the start
    //  // state to a new pose.





    //  // set the grippers to the default position

    //  actionlib::SimpleActionClient<franka_gripper::MoveAction> ac("franka_gripper_node/move",true);


    //  // wait for the Move action client server to start

    //  ac.waitForServer(); //will wait for infinite time

    //  ROS_INFO("Action server started");


    //  // send a goal to the action

    //  franka_gripper::MoveGoal def;

    //  def.speed = 0.1;
    //  def.width = 0.08;

    //  ac.sendGoal(def);


    //  std::vector<geometry_msgs::Pose> grasp;

    //  geometry_msgs::Pose target_pose_1;

    //  target_pose_1.position.x = 0.42607;
    //  target_pose_1.position.y = 0.24535;
    //  target_pose_1.position.z = 0.65447;
    //  target_pose_1.orientation.x = 0.96178;
    //  target_pose_1.orientation.y = 0.27334;
    //  target_pose_1.orientation.z = -0.0097628;
    //  target_pose_1.orientation.w = 0.012971;


    //  grasp.push_back(target_pose_1); // default position


    //  target_pose_1.position.x = 0.51715;
    //  target_pose_1.position.y = 0.33304;
    //  target_pose_1.position.z = 0.13919;
    //  target_pose_1.orientation.x = 0.95649;
    //  target_pose_1.orientation.y = 0.29149;
    //  target_pose_1.orientation.z = -0.0064835;
    //  target_pose_1.orientation.w = -0.010645;


    //  grasp.push_back(target_pose_1);  // Point A


    //  // We want the cartesian path to be interpolated at a resolution of 1 cm
    //  // which is why we will specify 0.01 as the max step in cartesian
    //  // translation.  We will specify the jump threshold as 0.0, effectively
    //  // disabling it.



    //  moveit_msgs::RobotTrajectory trajectory_1;
    //  double fraction_1 = group.computeCartesianPath(grasp,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory_1);

    //  moveit::planning_interface::MoveGroup::Plan grasp_plan;
    //  grasp_plan.trajectory_ = trajectory_1;


    //  ROS_INFO("Visualizing cartesian path one (%.2f%% acheived)",
    //        fraction_1 * 100.0);

    //  // Excute first cartesian path

    ////  group.execute(grasp_plan);


    //  ros::Duration(0.1).sleep();


    //  actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_1("franka_gripper_node/grasp",true);

    //  ac_1.waitForServer(); //will wait for infinite time

    //  ROS_INFO("Action server started");


    //  franka_gripper::GraspGoal gripper_grasp;

    //  gripper_grasp.speed = 0.1;
    //  gripper_grasp.width = 0.05;
    //  gripper_grasp.force = 0.1;

    //  ac_1.sendGoal(gripper_grasp);

    //  while(!ac_1.getState().isDone())
    //  {
    //    // just looping
    //    ros::Duration(0.1).sleep();

    //  }

    //  std::vector<geometry_msgs::Pose> release;

    //  geometry_msgs::Pose target_pose_2;

    //  target_pose_2.position.x = 0.61052;
    //  target_pose_2.position.y = 0.057081;
    //  target_pose_2.position.z = 0.4377;
    //  target_pose_2.orientation.x = 0.99362;
    //  target_pose_2.orientation.y = 0.11048;
    //  target_pose_2.orientation.z = -0.022561;
    //  target_pose_2.orientation.w = -0.0026564;

    //  release.push_back(target_pose_2);  // Point B

    //  target_pose_2.position.x = 0.78473;
    //  target_pose_2.position.y = -0.11924;
    //  target_pose_2.position.z = 0.15584;
    //  target_pose_2.orientation.x = 0.95616;
    //  target_pose_2.orientation.y = 0.29148;
    //  target_pose_2.orientation.z = 0.025624;
    //  target_pose_2.orientation.w = -0.011784;

    //  release.push_back(target_pose_2);  // Point C


    //  moveit_msgs::RobotTrajectory trajectory_2;
    //  double fraction_2 = group.computeCartesianPath(release,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory_2);

    //  moveit::planning_interface::MoveGroup::Plan release_plan;
    //  release_plan.trajectory_ = trajectory_2;

    //  ROS_INFO("Visualizing cartesian path two (%.2f%% acheived)",
    //        fraction_2 * 100.0);

    //// Excute Second cartesian path


    ////  group.execute(release_plan);

    //  ros::Duration(0.1).sleep();

    //  ac.sendGoal(def);


    //  while (!ac.getState().isDone());

    //  {

    //      //just loop
    //      ros::Duration(0.1).sleep();
    //  }

    //  std::vector<geometry_msgs::Pose> end;

    //  geometry_msgs::Pose target_pose_def;


    //  target_pose_def.position.x = 0.42607;
    //  target_pose_def.position.y = 0.24535;
    //  target_pose_def.position.z = 0.65447;
    //  target_pose_def.orientation.x = 0.96178;
    //  target_pose_def.orientation.y = 0.27334;
    //  target_pose_def.orientation.z = -0.0097628;
    //  target_pose_def.orientation.w = 0.012971;


    //  end.push_back(target_pose_def);  // Go Back To The Default Position

    //  moveit_msgs::RobotTrajectory trajectory_3;
    //  double fraction_3 = group.computeCartesianPath(end,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory_3);

    //  moveit::planning_interface::MoveGroup::Plan end_plan;
    //  end_plan.trajectory_ = trajectory_3;

    //  ROS_INFO("Visualizing cartesian path two (%.2f%% acheived)",
    //        fraction_3 * 100.0);

    //// Excute Second cartesian path

    ////  group.execute(end_plan);



    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // Now that we have a plan we can visualize it in Rviz.  This is not
    // necessary because the group.plan() call we made above did this
    // automatically.  But explicitly publishing plans is useful in cases that we
    // want to visualize a previously created plan.
    //  if (true)
    //  {
    //    ROS_INFO("Visualizing plan 1 (again)");
    //    display_trajectory.trajectory_start = my_plan.start_state_;
    //    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    //    display_publisher.publish(display_trajectory);
    //    /* Sleep to give Rviz time to visualize the plan. */
    //    sleep(5.0);
    //  }

    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // Moving to a pose goal is similar to the step above
    // except we now use the move() function. Note that
    // the pose goal we had set earlier is still active
    // and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is
    // a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.


    // -------------------------------------------------------------------------------------------------------------- //


    //  // Planning to a joint-space goal
    //  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //  //
    //  // Let's set a joint space goal and move towards it.  This will replace the
    //  // pose target we set above.
    //  //
    //  // First get the current set of joint values for the group.
    //  std::vector<double> group_variable_values;
    //  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    //  // Now, let's modify one of the joints, plan to the new joint
    //  // space goal and visualize the plan.
    //  group_variable_values[0] = -1.0;
    //  group.setJointValueTarget(group_variable_values);
    //  success = group.plan(my_plan);

    //  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
    //  /* Sleep to give Rviz time to visualize the plan. */
    //  sleep(5.0);

    //  // Planning with Path Constraints
    //  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //  //
    //  // Path constraints can easily be specified for a link on the robot.
    //  // Let's specify a path constraint and a pose goal for our group.
    //  // First define the path constraint.
    //  moveit_msgs::OrientationConstraint ocm;
    //  ocm.link_name = "r_wrist_roll_link";
    //  ocm.header.frame_id = "base_link";
    //  ocm.orientation.w = 1.0;
    //  ocm.absolute_x_axis_tolerance = 0.1;
    //  ocm.absolute_y_axis_tolerance = 0.1;
    //  ocm.absolute_z_axis_tolerance = 0.1;
    //  ocm.weight = 1.0;

    //  // Now, set it as the path constraint for the group.
    //  moveit_msgs::Constraints test_constraints;
    //  test_constraints.orientation_constraints.push_back(ocm);
    //  group.setPathConstraints(test_constraints);

    //  // We will reuse the old goal that we had and plan to it.
    //  // Note that this will only work if the current state already
    //  // satisfies the path constraints. So, we need to set the start
    //  // state to a new pose.
    //  robot_state::RobotState start_state(*group.getCurrentState());
    //  geometry_msgs::Pose start_pose;
    //  start_pose.orientation.w = 1.0;
    //  start_pose.position.x = 0.55;
    //  start_pose.position.y = -0.05;
    //  start_pose.position.z = 0.8;
    //  const robot_state::JointModelGroup *joint_model_group =
    //                  start_state.getJointModelGroup(group.getName());
    //  start_state.setFromIK(joint_model_group, start_pose);
    //  group.setStartState(start_state);

    //  // Now we will plan to the earlier pose target from the new
    //  // start state that we have just created.
    //  group.setPoseTarget(target_pose1);
    //  success = group.plan(my_plan);

    //  ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
    //  /* Sleep to give Rviz time to visualize the plan. */
    //  sleep(10.0);

    //  // When done with the path constraint be sure to clear it.
    //  group.clearPathConstraints();

    //  // Cartesian Paths
    //  // ^^^^^^^^^^^^^^^
    //  // You can plan a cartesian path directly by specifying a list of waypoints
    //  // for the end-effector to go through. Note that we are starting
    //  // from the new start state above.  The initial pose (start state) does not
    //  // need to be added to the waypoint list.

    //  std::vector<geometry_msgs::Pose> waypoints;

    //  geometry_msgs::Pose target_pose3 = start_pose;
    //  target_pose3.position.x += 0.2;
    //  target_pose3.position.z += 0.2;
    //  waypoints.push_back(target_pose3);  // up and out

    //  target_pose3.position.y -= 0.2;
    //  waypoints.push_back(target_pose3);  // left

    //  target_pose3.position.z -= 0.2;
    //  target_pose3.position.y += 0.2;
    //  target_pose3.position.x -= 0.2;
    //  waypoints.push_back(target_pose3);  // down and right (back to start)

    //  // We want the cartesian path to be interpolated at a resolution of 1 cm
    //  // which is why we will specify 0.01 as the max step in cartesian
    //  // translation.  We will specify the jump threshold as 0.0, effectively
    //  // disabling it.

    //  moveit_msgs::RobotTrajectory trajectory;
    //  double fraction = group.computeCartesianPath(waypoints,
    //                                               0.01,  // eef_step
    //                                               0.0,   // jump_threshold
    //                                               trajectory);

    //  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
    //        fraction * 100.0);
    //  /* Sleep to give Rviz time to visualize the plan. */
    //  sleep(15.0);



    //  // Planning with collision detection can be slow.  Lets set the planning time
    //  // to be sure the planner has enough time to plan around the box.  10 seconds
    //  // should be plenty.
    //  group.setPlanningTime(10.0);

    //  // Now, let's attach the collision object to the robot.
    //  ROS_INFO("Attach the object to the robot");
    //  group.attachObject(collision_object.id);
    //  /* Sleep to give Rviz time to show the object attached (different color). */
    //  sleep(4.0);


    //  // Now, let's detach the collision object from the robot.
    //  ROS_INFO("Detach the object from the robot");
    //  group.detachObject(collision_object.id);
    //  /* Sleep to give Rviz time to show the object detached. */
    //  sleep(4.0);


    //  // Now, let's remove the collision object from the world.
    //  ROS_INFO("Remove the object from the world");
    //  std::vector<std::string> object_ids;
    //  object_ids.push_back(collision_object.id);
    //  planning_scene_interface.removeCollisionObjects(object_ids);
    //  /* Sleep to give Rviz time to show the object is no longer there. */
    //  sleep(4.0);



    ////  // Now when we plan a trajectory it will avoid the obstacle
    //  group.setStartState(*group.getCurrentState());
    //  group.setPoseTarget(target_pose1);
    //  success = group.plan(my_plan);

    //  ROS_INFO("Visualizing plan 5 (pose goal move around table) %s",
    //    success?"":"FAILED");
    //  /* Sleep to give Rviz time to visualize the plan. */
    //  sleep(10.0);



    //  // Dual-arm pose goals
    //  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //  // First define a new group for addressing the two arms. Then define
    //  // two separate pose goals, one for each end-effector. Note that
    //  // we are reusing the goal for the right arm above
    //  moveit::planning_interface::MoveGroup two_arms_group("arms");

    //  two_arms_group.setPoseTarget(target_pose1, "r_wrist_roll_link");

    //  geometry_msgs::Pose target_pose;
    //  target_pose.orientation.w = 1.0;
    //  target_pose.position.x = 0.7;
    //  target_pose.position.y = 0.15;
    //  target_pose.position.z = 1.0;

    //  two_arms_group.setPoseTarget(target_pose, "l_wrist_roll_link");

    //  // Now, we can plan and visualize
    //  moveit::planning_interface::MoveGroup::Plan two_arms_plan;
    //  two_arms_group.plan(two_arms_plan);
    //  sleep(4.0);

    // END_TUTORIAL

    //  ROS_INFO("Going to sleep, wake me up by hitting ctrl+c");
    //  while (ros::ok())
    //  {
    //    ros::Duration(0.5).sleep();
    //  }

    //  actionlib::SimpleActionClient<franka_gripper::StopAction> ac_s("franka_gripper_node/stop",true);

    //  ac_s.waitForServer(); //will wait for infinite time

    //  ROS_INFO("Action server started");


    //  franka_gripper::StopGoal end;

    //  ac_s.sendGoal(end);

    // Set Franka external load

    //     ros::ServiceClient load = node_handle.serviceClient<franka_control::SetLoad>("/panda/set_load");

    //     franka_control::SetLoad srv_2;

    //     srv_2.request.F_x_center_load.assign(20.0);
    //     srv_2.request.load_inertia.assign(30.0);
    //     srv_2.request.mass = 10.0;

    //   //  load.call(srv_2);


    ros::shutdown();
    return 0;
}
