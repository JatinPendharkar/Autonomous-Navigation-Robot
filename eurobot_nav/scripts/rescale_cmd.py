#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import copysign

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def cmd_vel_cb(msg):
    
    #if (msg.linear.x > 0.12):
        #msg.angular.z = 0.0
     #   msg.linear.x = 0.75
    #elif (msg.angular.z < 0.4) and (msg.linear.x > 0.5):
    	#msg.angular.z = 0.0
    
    msg.linear.x = 3*msg.linear.x
    if (msg.linear.x > 0.4) and (msg.angular.z < 0.5):
    	msg.angular.z = 0.0
    if (msg.linear.x < 0.2) and (abs(msg.angular.z) > 0.25):
    	msg.angular.z = copysign(1.5, msg.angular.z)
    pub.publish(msg)

def main():
    rospy.init_node('rescale_cmd')
    rospy.Subscriber('/cmd_vel_raw', Twist, cmd_vel_cb)
    rospy.spin()

if __name__ == '__main__':
    main()

