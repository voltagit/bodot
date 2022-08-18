#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import sys
import signal

import rospy
from std_msgs.msg import Int64

encL_a = 12
encL_b = 16
encR_a = 20
encR_b = 21 

pulse_L = 0
pulse_R = 0
pulse_L_state = 0

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(encL_a, GPIO.IN)
GPIO.setup(encR_a, GPIO.IN)
GPIO.setup(encL_b, GPIO.IN)
GPIO.setup(encR_b, GPIO.IN)

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def encL_a_callback(channel):
    global pulse_L
    if GPIO.input(encL_b) == 0:
        if GPIO.input(encL_a) == 0:
            pulse_L-=1
        else:
            pulse_L+=1
    print("pulse_R:          ", pulse_R, "         pulse_L:          ", pulse_L)

def encR_a_callback(channel):
    global pulse_R
    if GPIO.input(encR_b) == 0:
        if GPIO.input(encR_a) == 0:
            pulse_R-=1
        else:
            pulse_R+=1
    print("pulse_R:          ", pulse_R, "         pulse_L:          ", pulse_L)

if __name__ == '__main__':
    encL_pub = rospy.Publisher('/enc_L', Int64, queue_size=1)
    encR_pub = rospy.Publisher('/enc_R', Int64, queue_size=1)
    while not rospy.is_shutdown():

        GPIO.add_event_detect(encR_a, GPIO.BOTH, 
                callback=encR_a_callback, bouncetime=1)
        GPIO.add_event_detect(encL_a, GPIO.BOTH, 
                callback=encL_a_callback, bouncetime=1)
        signal.signal(signal.SIGINT, signal_handler)
        
        encL_pub.publish(pulse_L)
        encR_pub.publish(pulse_R)

        signal.pause()