# ur5_jog_arm
The package contains custom configuration for our specific ur5 configuration and launch files.

## Running the jogger

### Required packages

Pull and build the packages in your catkin workspace: 

1. https://github.com/UTNuclearRoboticsPublic/jog_arm.git - Main jogger logic. Reccomended reading: https://wiki.ros.org/jog_arm

2. https://github.com/ut-ims-robotics/ur5_force_control - (current repository) contains ur5_jog_arm package, which contains custom configuration for our specific ur5 configuration and launch files.

3. https://github.com/ut-ims-robotics/keyboard_publisher - keyboard publisher, for testing the jogger with keyboard

4. https://github.com/ut-ims-robotics/ur5_path_planning - and every package that is described in readme of this repository

### Starting the jogger test scripts

1. First of all, everything that is needed for Moveit! path-planning should be running. Follow instructions of ur5_path_planning repository.

2. Now currently, to test jogger, there are two options: 1. try it with controller/joystick, 2. try it with keyboard (W, A, S, D)

    2.1 ```$ roslaunch ur5_jog_arm test_with_joystick.launch```

    2.2 ```$ roslaunch ur5_jog_arm test_with_keyboard.launch```
    
### Your custom control method

To use your own control method, you can see the joystick and keyboard examples and interpret from these. Correspondingly key_to_twist_node.py from keyboard_publisher package or joy_to_twist.py from jog_arm package.