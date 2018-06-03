#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String


class MoveRobot():

    def __init__(self):
        self.incomingData = -1

    def callback(self, data):
        self.lastDataTime = rospy.get_time()
        try:
            self.incomingData = int(data.data)
        except ValueError:
            self.incomingData = -1
            # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

    def ur5_openvibe_pose(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur5', anonymous=True)
        self.lastDataTime = rospy.get_time()

        sub = rospy.Subscriber('chatter', String, self.callback)  # Get input from openvibe_to_ros_tcp
        rate = rospy.Rate(50)

        # Instantiate a RobotCommander object. This object is an interface to the robot as a whole
        robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints
        group = moveit_commander.MoveGroupCommander("manipulator")  # UR5 reference name

        # Create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Get the name of the reference frame for this robot
        # print "============ Reference frame: %s" % group.get_planning_frame()

        # Print the name of the end-effector link for this group
        # print "============ End effector: %s" % group.get_end_effector_link()

        # Sometimes for debugging it is useful to print the entire state of the robot
        # print "============ Printing robot state"
        # print robot.get_current_state()
        # print "============"

        print "Going to HOME position"
        # Plan a motion for this group to a desired pose for the end-effector
        # pose_target = geometry_msgs.msg.Pose()
        # pose_target.orientation.w = 0.1
        # pose_target.position.x = 0.3
        # pose_target.position.y = 0.3
        # pose_target.position.z = 0.3
        # group.set_pose_target(pose_target)

        # Use named target
        group.set_named_target("home")
        plan1 = group.plan()

        # Move robot
        group.go(wait=True)

        group.clear_pose_targets()

        dataTimeout = 1

        # Modify one of the joints
        joint_increment = 0.5
        while not rospy.is_shutdown():
            now = rospy.get_time()
            # rospy.loginfo('lastDataTime %s and now %s', str(self.lastDataTime), str(now))

            if (now - self.lastDataTime) > dataTimeout:
                self.incomingData = -1
            # rospy.loginfo('I got %d', self.incomingData)

            group_variable_values = group.get_current_joint_values()
            if self.incomingData == 1:  # Up
                group_variable_values[2] += joint_increment
            elif self.incomingData == 2:  # Left
                group_variable_values[0] -= joint_increment
            elif self.incomingData == 3:  # Right
                group_variable_values[0] += joint_increment
            elif self.incomingData == 4:  # Down
                group_variable_values[2] -= joint_increment

            # Doesn't work properly if value is larger or smaller than pi
            # TODO: transform targets appropriately when larger or smaller than pi
            for element in group_variable_values:
                if element > 3.14:
                    element = 3.14
                if element < -3.14:
                    element = -3.14

            group.set_joint_value_target(group_variable_values)
            group.go(wait=True)

            rate.sleep()
        # Exit with Ctrl + C
        print "STOPPING"


if __name__ == '__main__':
    try:
        myRobot = MoveRobot()
        myRobot.ur5_openvibe_pose()

    except rospy.ROSInterruptException:
        pass
