#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import sys
import signal
from time import sleep

import rospy
from std_msgs.msg import Int32

class MotorDriver():
    def __init__(self):
        self.AN1 = 26  # pin PWM
        self.DIG1 = 13  # pin DIR
        self.AN2 = 24  # pin PWM
        self.DIG2 = 23  # pin DIR
        self.freq = 1000
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
            '''self.p1.start(20)
            self.p2.start(20)
            GPIO.output(self.DIG1, GPIO.LOW)
            GPIO.output(self.DIG2, GPIO.LOW)
            self.p1.ChangeDutyCycle(self.pwmL.data)
            self.p2.ChangeDutyCycle(self.pwmR.data)'''
            if self.pwmL.data<0:
                GPIO.output(self.DIG1, GPIO.LOW)
            if self.pwmR.data<0:
                GPIO.output(self.DIG2, GPIO.LOW)
            if self.pwmL.data>0:
                GPIO.output(self.DIG1, GPIO.HIGH)
            if self.pwmR.data>0:
                GPIO.output(self.DIG2, GPIO.HIGH)
            self.p1.ChangeDutyCycle(abs(self.pwmL.data))
            self.p2.ChangeDutyCycle(abs(self.pwmR.data))
            sleep(0.01)
            
        self.p1.stop()
        self.p2.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    rospy.init_node("motor_driver")
    MD_object = MotorDriver()
    MD_object.run_motor_driver()
    


