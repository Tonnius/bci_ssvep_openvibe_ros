# VEP-based BCI using OpenVibe and ROS
## Introduction
This repository was initially produced in the process of an MSc thesis at the University of Tartu. The main purpose of this repository is to give an overview of the developed **OpenVibe** scenarios and **ROS** packages for setting up a brain-computer interface (BCI) for controlling the UR5 and Franka Emika Panda robots using the **[OpenBCI Cyton/Ultracortex EEG headset](http://openbci.com/)**. The whole system was set up on Ubuntu 16.04 because of ROS, but the BCI software on OpenVibe can be used on any other OS. 

**Example videos of the system in use: [Video 1](https://youtu.be/jVh7MMvXQok), [Video 2](https://youtu.be/asDwupMbE2I)**

## OpenVibe set up

### OpenVibe installation
* Linux - download the OpenVibe [source code](http://openvibe.inria.fr/downloads/) and see [build instructions](http://openvibe.inria.fr/build-instructions/) for details.
* Windows - download [Windows installer](http://openvibe.inria.fr/downloads/).

### OpenVibe Acquisition server
Run OpenVibe Acquisition server (either *openvibe-acquisition-server.sh* in Linux or .cmd in Windows) with the OpenBCI driver. More info [here](http://docs.openbci.com/3rd%20Party%20Software/03-OpenViBE).

### OpenVibe Designer
To start, clone this repository with submodules (`git clone --recursive`).

The OpenVibe folder of this repository contains all the neccessary files for using the BCI part of the system. Once OpenVibe is installed, run *openvibe-designer.sh* or .cmd and open the scenarios files (.mxs) in OpenVibe. The following is a list of the most important scenarios and the order that they should be run:

1. **ssvep-configuration.mxs** - for setting system parameters
2. **training-acquisition.mxs** - for training data acquisition
3. **CSP-training-harm.mxs** - for training the CSP spatial filters
4. **classifier-training-harm.mxs** - for training the classifiers
5. **online-4-stim.mxs** - for using the system online with 4 stimuli

Also included are *acquisition-test.mxs* for viewing the EEG signals before acquisition and *perf-measure-harm.mxs* for performance measurements on a test dataset.

A training and test dataset is provided for one subject as an example (ssvep-record-subjectX.ov) where the subject was shown SSVEP stimuli with frequencies 20, 15, 12, 10 Hz. If scenarios 1 - 4 are completed successfully, running the online-4-stim scenario will classify the visual stimulus that the user is watching and send commands to ROS via TCP socket.

NOTE: by default, OpenVibe is set up to display 3 stimuli. To configure the number of stimuli, the files *openvibe-ssvep-demo.conf* and *trainer.conf* in your OpenVibe installation directory must be edited. Examples for 4 stimuli are in the bci_ssvep_openvibe_ros/OpenVibe/ssvep-demo folder of this repository.

## ROS setup
You should already have ROS Kinetic installed and catkin workspace created. 

1. Copy the contents of the ROS folder in this repository to your catkin workspace src folder (`~/../my_workspace/src`)
2. Install all dependencies: `~/../my_workspace$ rosdep install --from-paths --ignore-src .`
3. Compile: `~/../my_workspace$ catkin_make`

If dependency errors or compiler errors occur at this point, look at the documentation of the git submodules. If no errors occur, continue to robot-specific packages.

### UR5
An introductory tutorial for using the UR5 can be found [here](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial). For moving the robot in two axes (up, down, left, right) run the following:

1. `$ roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=<ROBOT_IP>` - robot bring-up, where ROBOT_IP is the IP address of the robot.

2. `$ roslaunch ur5_custom_config ur5_on_table_moveit_planning_execution.launch` - loads configuration files and a model where the robot is on a table.

3. `$ roslaunch ur5_openvibe_move ur5_jog.launch` - launches jogging server for moving the robot in small incremental steps with velocity control.

   OR  

   `$ roslaunch ur5_openvibe_move ur5_move_pose.launch` - moving the robot via incremental poses.  

   (Optional): `roslaunch ur5_jog_arm test_with_keyboard.launch` - move the robot with a keyboard (W, A, S, D).  

4. `$ rosrun openvibe_to_ros_tcp client_node <SERVER_IP_ADDRESS> <PORT>` - TCP socket client node for interfacing OpenVibe and ROS. In the current setup SERVER_IP_ADDRESS = localhost, PORT = 5678. NOTE: the client_node will run properly if the TCP server has been started in OpenVibe (by running online-4-stim.mxs) and roscore has been started (any of the previous launch files does that).

### Franka Emika Panda
The particular setup used the Kinova KG-3 gripper on the Franka Emika Panda. For properly interfacing with the gripper, the [Kinova K-Series SDK](https://drive.google.com/file/d/1dFKkJeGiRlSAabhaQTuiR6M_zAxXDcI7/view) must be installed. An introductory tutorial for using the Panda can be found [here](https://github.com/ut-ims-robotics/tutorials/wiki/Franka-Emika-Panda-beginner-guide).

Using the Franka Emika Panda robotic arm and Kinova gripper for position control (grasping an object the user picked):

1. `$ roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=c1n4s300 use_urdf:=false` - only for starting up the Kinova gripper.
2. `$ roslaunch franka_openvibe_move franka_control_ed.launch robot_ip:=<ROBOT_IP> load_gripper:=false` - Panda robot bring-up. load_gripper:=false means that the default Franka gripper is ignored and the Kinova gripper is used. 
3. `$ roslaunch franka_kinova_movit franka_kinova.launch` - loads configuration files and a model where the robot is on a table.
4. `$ roslaunch franka_openvibe_move panda.launch` - robot control logic.

   (Optional): `$ roslaunch keyboard_publisher forward_keys.launch` - control the robot with a keyboard (panda.launch must be running).  
5. `$ rosrun openvibe_to_ros_tcp client_node <SERVER_IP_ADDRESS> <PORT>` - OpenVibe-ROS interfacing, same as for UR5.
