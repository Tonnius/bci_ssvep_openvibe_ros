# ur5_path_planning
General package for using UR5 robot with custom moveit config

# Getting started

Following are step-by-step points for connecting UR5 robot with ROS.

## Hardware setup:

1. UR5 robot should be connected to your computer through router.

2. From robot's dashboard, check its network properties. "setup robot" -> "Network". You should see an IP address there (ROBOT_IP for further reference). First, try to ping corresponding IP. If it succeeds, then your computer is connected.


## Software:

1. Pull this repository to your workspace. 

2. Now, you should install required dependencies. Required packages for running the robot are following:
 
 * ur_modern_driver - https://github.com/ThomasTimm/ur_modern_driver
 * universal_robot - https://github.com/ros-industrial/universal_robot.git

To use these packages, you should pull them to your workspace and catkin_make the project.

3. Start the ur_modern_driver. ROBOT_IP is reference to the IP you configured previously in "Hardware setup" section.

```
$ roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=ROBOT_IP
```

4. Start moveit planning execution by executing
```
$ roslaunch ur5_custom_config ur5_on_table_moveit_planning_execution.launch
```

5. Start moveit rviz

```
$ roslaunch ur5_custom_config moveit_rviz.launch config:=true
```
