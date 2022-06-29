#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
import numpy as np
#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/

#Constants
nbPCAServo=16

joints = {
        'servo0': {
            'chan': 0,
            'min_imp': 500,
            'max_imp': 2500,
            'min_ang': 0,
            'max_ang': 180
            },
        'servo1': {
            'chan': 1,
            'min_imp': 500,
            'max_imp': 2500,
            'min_ang': 0,
            'max_ang': 180
            },
 
        }



#Parameters
NAMES  = ['servo1', 'servo2', ]
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]

#Objects
pca = ServoKit(channels=16)

# function init 

def init_pca():

    for k, v in joints.items():
        print(k,  v)
        c = v['chan']
        pca.servo[c].set_pulse_width_range(MIN_IMP[c] , MAX_IMP[c])
        pca.servo[c].angle=None
           

#As for the joint trajectory controller I am not sure what the question is but here's a simple example: a joint position is requested, the joint is set up with limits. A spline interpolation (there are a few available in the controller) is used to generate a one-dof joint trajectory. Joint positions are defined in angles for rotational and meters for linear actuators. This is transformed into a specific servo position by your RobotHW implementation, aka the Hardware Abstraction Layer.
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data)
    print(data.position[0])
    for k in data.name:
        idx = data.name.index(k)
        #deg = np.rad2deg(np.pi / data.position[idx])
        # FIXME: figure out what is the negative values (doesn't work with negative) 
        deg = np.rad2deg(data.position[idx])


        print(str(idx) + ' ' + str(k) + ' pos: ' + str(data.position[idx]) + ' degrees: ' + str(deg))
        pca.servo[joints[k]['chan']].angle=deg


def listener():
    init_pca()
    rospy.init_node('servo', anonymous=True)
    rospy.Subscriber("commands", JointState, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        rospy.logfatal("Node crashed due to an internal exception")





