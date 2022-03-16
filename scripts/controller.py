#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2


import math
x = 0.0
y = 0.0
theta = 0.0


dist_precision_ = 0.3

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node('speed_controller')
sub = rospy.Subscriber("/odom",Odometry,newOdom)
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = 0
goal.y = -5

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_to_goal= atan2(inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
       
    
        desired_yaw = math.atan2(goal.y - y, goal.x - x)
        err_yaw = desired_yaw - theta
        err_pos = math.sqrt(pow(goal.y - y, 2) + pow(goal.x - x, 2))
       

        if err_pos > dist_precision_:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
          

        else:
            print ('Position error: [%s]' % err_pos)
            speed.linear.x = 0.0
            speed.linear.y = 0.0
            speed.angular.z = 0.0

         

    pub.publish(speed)
    r.sleep()