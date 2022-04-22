#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import numpy as np
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf import transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped

class OdomCalculate:
    def __init__(self):
        rospy.init_node("odom_calculate_node")

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=30)
        l_encoder_sub = rospy.Subscriber('FL_sub', Int32, self.left_enc_cb)
        r_encoder_sub = rospy.Subscriber('FR_sub', Int32, self.right_enc_cb)

        self.fl_encoder = 0.0
        self.fr_encoder = 0.0

        self.enc_left = 0.0
        self.enc_right = 0.0

        self.ticks_meter = 195 #830 # 790 #2000 # 4126 #4372 #4643  #5170
        self.base_width = 0.23 #0.475 #0.6 #0.475 #0.419103 #0.325 #0.35

        self.x_final = 0.0
        self.y_final = 0.0
        self.theta_final = 0.0
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = "odom"
        self.t.child_frame_id = "base_link"

        self.odom = Odometry()
        self.odom.child_frame_id = 'base_link'
        self.odom.header.frame_id = 'odom'

    def left_enc_cb(self, msg):
        self.fl_encoder = msg.data

    def right_enc_cb(self, msg):
        self.fr_encoder = msg.data
        

    def update(self):
        left = self.fl_encoder
        right = self.fr_encoder

        d_left = (left - self.enc_left) / self.ticks_meter
        d_right = (right - self.enc_right) / self.ticks_meter

        self.enc_left = left
        self.enc_right = right


        d = (d_left + d_right) / 2.0

        th = (d_right - d_left) / self.base_width

        if d != 0.0:
            x = np.cos(th) * d
            y = -np.sin(th) * d

            self.x_final = self.x_final + ( np.cos( self.theta_final ) * x - np.sin( self.theta_final ) * y )
            self.y_final = self.y_final + ( np.sin( self.theta_final ) * x + np.cos( self.theta_final ) * y )

        if th != 0.0:
            self.theta_final = self.theta_final - th

        
        quaternion = transformations.quaternion_from_euler(0, 0, self.theta_final)

        self.odom.pose.pose.position.x = self.x_final
        self.odom.pose.pose.position.y = self.y_final
        self.odom.pose.pose.orientation.x = quaternion[0]
        self.odom.pose.pose.orientation.y = quaternion[1]
        self.odom.pose.pose.orientation.z = quaternion[2]
        self.odom.pose.pose.orientation.w = quaternion[3]
        self.t.transform.translation.x = self.x_final
        self.t.transform.translation.y = self.y_final
        self.t.transform.rotation.x = quaternion[0]
        self.t.transform.rotation.y = quaternion[1]
        self.t.transform.rotation.z = quaternion[2]
        self.t.transform.rotation.w = quaternion[3]
        

        self.odom.header.seq = rospy.Time.now()
        self.t.header.stamp = rospy.Time.now()

        self.odom_pub.publish(self.odom)
        self.br.sendTransform(self.t)

    def run(self):
        '''
        spin node
        '''
        rate = rospy.Rate(30) # 30 hz
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()



def main():
    '''
    main function
    '''
    odom_calculate = OdomCalculate()
    odom_calculate.run()



if __name__ == '__main__':
    main()
