#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
import numpy as np
#Libraries
import time    #https://docs.python.org/fr/3/library/time.html

def main():
    # Initialize the node
    rospy.init_node("Py_JointStates")
    # Publisher object
    publisherObject = rospy.Publisher("commands", JointState, queue_size=10)
    # Rate controller
    rateController = rospy.Rate(20)
    # Variables
    msg = JointState()
    msg.header.frame_id = ""
    msg.name = ["servo0"]       # Joint name (array assignment)
    angle = 0
    increment = True
    # Main loop
    while(not rospy.is_shutdown()):
        # Initialize the time of publishing
        msg.header.stamp = rospy.Time.now()
        # Joint angle values
        msg.position = [angle]
        # Publish message
        publisherObject.publish(msg)
        print(msg)
        # Increase sequence
        msg.header.seq += 1
        # Change angle value
        if increment:
            #angle += 90.0/20.0 * math.pi / 180.0
            angle += math.radians(5)
        else:
            #angle -= 90.0/20.A0 * math.pi / 180.0
            angle -= math.radians(5)
        #if angle > math.pi or angle < - math.pi:
        if angle > math.pi or angle < 0:
            increment = not increment
        # Delay execution to match rate
        rateController.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        rospy.logfatal("Node crashed due to an internal exception")





