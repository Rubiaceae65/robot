import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0
y = 0 
theta = 0

goal = Point()
goal.x = 5
goal.y = 5

def newOdom(msg):
    global x
    global y
    global theta

    print("new odom: " + msg)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    inc_x = goal.x -x
    inc_y = goal.y -y

    angle_to_goal = atan2(inc_y, inc_x)

    if abs(angle_to_goal - theta) < 1:  
        speed.linear.x = 0.0
        speed.angular.z = 0.6
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    print('publsi \n' + str(angle_to_goal) + ' \n ' + str(speed))
    pub.publish(speed)

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/base_controller/odom", Odometry, newOdom)
pub = rospy.Publisher("/base_controller/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)


while not rospy.is_shutdown():
    r.sleep() 