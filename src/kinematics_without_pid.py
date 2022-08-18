#! /usr/bin/env python3

import rospy
from math import pi
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class ConvertVelocities():
    def __init__(self):
        self.R = 0.06 #radius of wheel
        self.L = 0.308 #distance between wheels
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        self.vL_pub = rospy.Publisher('/pwmL_without_pid/command', Int32, queue_size=1)
        self.vR_pub = rospy.Publisher('/pwmR_without_pid/command', Int32, queue_size=1)
        self.cmd_vel = Twist()
        self.rate = rospy.Rate(1)

    def callback(self, msg):
        self.cmd_vel = msg

    def convert_vel(self):
        self.vL = self.cmd_vel.linear.x - (self.cmd_vel.angular.z*(self.L/2))
        self.vR = self.cmd_vel.linear.x + (self.cmd_vel.angular.z*(self.L/2))

        self.pwmL = int(self.vL*5);
        self.pwmR = int(self.vR*5);

    def pub_rpm(self):
        while not rospy.is_shutdown():
            self.convert_vel()
            self.vL_pub.publish(self.pwmL)
            self.vR_pub.publish(self.pwmR)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("kinematic")
    CV_object = ConvertVelocities()
    CV_object.pub_rpm()
