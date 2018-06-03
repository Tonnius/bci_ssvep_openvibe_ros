# VEP-based BCI using OpenVibe and ROS
## Introduction
This repository was initially produced in the process of an MSc thesis at the University of Tartu. The main purpose of this repository is to give an overview of the developed OpenVibe and ROS packages for setting up the brain-computer interface (BCI) for robot control. The whole system was set up on Ubuntu 16.04 because of ROS, but the BCI part on OpenVibe can be tested on any other OS. 

## OpenVibe installation
* Linux - download the OpenVibe [source code](http://openvibe.inria.fr/downloads/) and see [build instructions](http://openvibe.inria.fr/build-instructions/) for details.
* Windows - download [Windows installer](http://openvibe.inria.fr/downloads/).

## OpenVibe set up

### OpenVibe Openvibe acquisition server
Run Openvibe acquisition server (either openvibe-acquisition-server.sh in Linux or .cmd in Windows) with the OpenBCI driver. More info [here](http://docs.openbci.com/3rd%20Party%20Software/03-OpenViBE).

### OpenVibe Designer
To start, clone this repository with submodules (`git clone --recursive`).

The OpenVibe folder of this repository contains all the neccessary files for using the BCI part of the system. Once OpenVibe is installed run openvibe-designer.sh or .cmd and open the scenarios files (.mxs) in OpenVibe. The following is a list explaining the most important scenarios and the order they should be used:

1. ssvep-configuration.mxs - for setting up system parameters
2. training-acquisition.mxs - for training data acquisition
3. CSP-training-harm.mxs - for training the CSP spatial filters
4. classifier-training-harm.mxs - for training the classifiers
5. online-4-stim.mxs - for using the system online with 4 stimuli

If steps 1 - 4 were completed successfully, running the online-4-stim scenario will classify the visual stimuli that the user is watching and send a command to ROS via TCP socket.

NOTE: by default, OpenVibe is set up to display 3 stimuli. To configure the number of stimuli, the files *openvibe-ssvep-demo.conf* and *trainer.conf* in your OpenVibe installation directory must be edited. Examples for 4 stimuli are in the bci_ssvep_openvibe_ros/OpenVibe/ssvep-demo folder of this repository.

## ROS setup
You should already have ROS Kinetic installed and catkin workspace created. 

1. Copy the contents of the ROS folder in this repository to your catkin workspace src folder (`~/../my_workspace/src`):
2. Install all dependencies: `~/../my_workspace$ rosdep install --from-paths --ignore-src .`
3. Compile: `~/../my_workspace$ catkin_make`

If dependency errors or compiler errors occur at this point, look at the documentation of the added git submodules. If no errors occur, connect to the robot (WiFi or Ethernet) and continue to robot-specific packages.

### UR5
For moving the robot in two axes (up, down, left, right) run the following:

1. `$ roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=<ROBOT_IP>` - robot bring-up, where ROBOT_IP is the ip address of the robot.
2. `$ roslaunch ur5_custom_config ur5_on_table_moveit_planning_execution.launch` - loads configuration files and robot model where the robot is on a table.
3. `$ roslaunch ur5_jog_arm test_with_openvibe.launch` (launches jogging server for moving the robot in small incremental steps with velocity control) OR `$ roslaunch ur5_openvibe_move move_ur5.launch` (moving the robot via incremental poses).
(Optional): `roslaunch ur5_jog_arm test_with_keyboard.launch` - to test the robot with a keyboard (either test_with_openvibe.launch or move_ur5.launch must be running).
4. `$ rosrun openvibe_to_ros_tcp client_node <SERVER_IP_ADDRESS> <PORT>` - client node for interfacing OpenVibe and ROS. In the current setup SERVER_IP_ADDRESS=localhost, PORT=5678. NOTE: the client_node will run properly if the TCP server has been started in OpenVibe (by running online-4-stim.mxs).

NOTE: The ROS/ur5/ur5_jog_arm package is a part of [this](https://github.com/ut-ims-robotics/ur5_force_control) UT IMS Robotics repository. No changes have been made to the original ur5_jog_arm package used in this repository.

### Franka Emika Panda
The particular setup used the Kinova KG-3 gripper on the Franka Emika Panda. For properly interfacing with the gripper, the [Kinova K-Series SDK](https://drive.google.com/file/d/1dFKkJeGiRlSAabhaQTuiR6M_zAxXDcI7/view) must be installed. 

Using the Franka Emika Panda robotic arm and Kinova gripper for position control (grasping an object the user picked):

1. `$ roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=c1n4s300 use_urdf:=false` - only for starting up the Kinova gripper.
2. `$ roslaunch franka_openvibe_move franka_control_ed.launch robot_ip:=<ROBOT_IP> load_gripper:=false` - Panda robot bring-up. load_gripper:=false means that the default Franka gripper is ignored and the Kinova gripper is used. 
3. `$ roslaunch franka_kinova_movit franka_kinova.launch` - loads configuration files and robot model where the robot is on a table.
4. `$ roslaunch franka_openvibe_move panda.launch` - robot position control logic
(Optional): `$ roslaunch keyboard_publisher forward_keys.launch` - control the robot with a keyboard (panda.launch must be running)
5. `$ rosrun openvibe_to_ros_tcp client_node <SERVER_IP_ADDRESS> <PORT>` - OpenVibe-ROS interfacing, same as for UR5.
