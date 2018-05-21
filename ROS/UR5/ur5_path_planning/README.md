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

 * ur_modern_driver:
    * ROS Indigo - https://github.com/ThomasTimm/ur_modern_driver
    * ROS Kinetic - fork from the same driver but with temporary patch for kinetic (full refactoring for kinetic is in progress at time of writing) - https://github.com/willcbaker/ur_modern_driver/tree/kinetic-devel 
 * universal_robot - https://github.com/ros-industrial/universal_robot.git

   2.1. You should clone the repositories to your workspace. Make sure you are on right ros distro branch, if corresponding branch is provided. 

   2.2. Download dependencies with rodsep
   ```
   [..]/your_workspace$ rosdep install --from-paths --ignore-src .
   ```

   2.3. make the project
   ```
   [..]/your_workspace$ catkin_make
   ```

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
