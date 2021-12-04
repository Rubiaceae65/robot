#!/usr/bin/python

import roslib
#roslib.load_manifest('rbx_nav')

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians

class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        square_size = 1.0 # meters
        turn_angle = radians(90) # degrees
        q_turn_angle = quaternion_from_euler(0, 0, turn_angle, axes='sxyz')
        quaternion_turn_angle = Quaternion()
        quaternion_turn_angle.x = q_turn_angle[0]
        quaternion_turn_angle.y = q_turn_angle[1]
        quaternion_turn_angle.z = q_turn_angle[2]
        quaternion_turn_angle.w = q_turn_angle[3]

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        rospy.loginfo("Starting navigation test")

        for i in range(4):
            # First move along a side
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose.position.x = 1.0
            self.goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)            
            self.goal.target_pose.header.frame_id = 'base_link'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move()

            # Now make a turn
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose.orientation = quaternion_turn_angle
            self.goal.target_pose.header.frame_id = 'base_link'
            self.goal.target_pose.header.stamp = rospy.Time.now()         
            self.move()  

    def move(self):
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)

            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 

            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
