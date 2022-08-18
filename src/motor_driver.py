#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import sys
import signal

import rospy
from std_msgs.msg import Int32

class MotorDriver():
    def __init__(self):
        self.AN1 = 23  # pin PWM
        self.DIG1 = 24  # pin DIR
        self.AN2 = 13  # pin PWM
        self.DIG2 = 26  # pin DIR
        self.freq = 980
        self.pwmL_sub = rospy.Subscriber('/pwmL_without_pid/command', Int32, self.pwmL_callback)
        self.pwmL = Int32()
        self.pwmR_sub = rospy.Subscriber('/pwmR_without_pid/command', Int32, self.pwmR_callback)
        self.pwmR = Int32()

    def pwmL_callback(self, msg):
        self.pwmL = msg 

    def pwmR_callback(self, msg):
        self.pwmR = msg 
    
    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.AN1, GPIO.OUT)              # set pin as output
        GPIO.setup(self.DIG1, GPIO.OUT)             # set pin as output
        GPIO.setup(self.AN2, GPIO.OUT)
        GPIO.setup(self.DIG2, GPIO.OUT)
        self.p1 = GPIO.PWM(self.AN1, self.freq)                  # set pwm for M1
        self.p1.start(0)
        self.p2 = GPIO.PWM(self.AN2, self.freq)                  # set pwm for M2
        self.p2.start(0)
    
    def run_motor_driver(self):
        self.setup_gpio()
        while not rospy.is_shutdown():
            GPIO.output(self.DIG1, GPIO.LOW)
            self.p1.ChangeDutyCycle(self.pwmL.data)
            GPIO.output(self.DIG2, GPIO.LOW)
            self.p2.ChangeDutyCycle(self.pwmR.data)
        self.p1.stop()
        self.p2.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    rospy.init_node("motor_driver")
    MD_object = MotorDriver()
    MD_object.run_motor_driver()
    


