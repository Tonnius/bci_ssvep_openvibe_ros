#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

class moveRobot():

  def __init__(self):
    self.incomingData = -1

  def callback(self, data):
    self.lastDataTime = rospy.get_time()
    try:
      self.incomingData = int(data.data)
    except ValueError:
      self.incomingData = -1
      #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

  def ur5_openvibe_pose(self):
   
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_openvibe_pose',
                    anonymous=True)
    self.lastDataTime = rospy.get_time()
    sub = rospy.Subscriber('chatter', String, self.callback)
    rate = rospy.Rate(50)

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.
    group = moveit_commander.MoveGroupCommander("manipulator")

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory,
                                        queue_size=20)

    ## We can get the name of the reference frame for this robot
    #print "============ Reference frame: %s" % group.get_planning_frame()

    ## We can also print the name of the end-effector link for this group
    #print "============ End effector: %s" % group.get_end_effector_link()

    ## Sometimes for debugging it is useful to print the entire state of the
    ## robot.
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print "============"


    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector
    print "Go to HOME position"
    #pose_target = geometry_msgs.msg.Pose()
    #pose_target.orientation.w = 0.1
    #pose_target.position.x = 0.3
    #pose_target.position.y = 0.3
    #pose_target.position.z = 0.3
    #group.set_pose_target(pose_target)
    
    group.set_named_target("home")
    plan1 = group.plan()
    rospy.sleep(1)
    
    # Use execute instead if you would like the robot to follow 
    # the plan that has already been computed
    #group.execute(plan1)

    # Or use the go() function
    group.go(wait=True)

    group.clear_pose_targets()
  
    dataTimeout = 1
    ## Now, let's modify one of the joints
    joint_increment = 1.0
    while not rospy.is_shutdown():
      now = rospy.get_time()
      #rospy.loginfo('lastDataTime %s and now %s', str(self.lastDataTime), str(now))

      if (now - self.lastDataTime) > dataTimeout: 
        self.incomingData = -1
      #rospy.loginfo('I got %d', self.incomingData)

      group_variable_values = group.get_current_joint_values()
      if self.incomingData == 1: 
        group_variable_values[2] += joint_increment
      elif self.incomingData == 2: 
        group_variable_values[0] -= joint_increment
      elif self.incomingData == 3: 
        group_variable_values[0] += joint_increment
      elif self.incomingData == 4:
        group_variable_values[2] -= joint_increment
      for element in group_variable_values:
        if element > 3.14: 
          element = 3.14
        if element < -3.14:
          element = -3.14

      group.set_joint_value_target(group_variable_values)
      group.go(wait=True)
     
      rate.sleep()

    print "STOPPING"


if __name__=='__main__':
  try:
    myRobot = moveRobot()
    myRobot.ur5_openvibe_pose()
    
  except rospy.ROSInterruptException:
    pass

