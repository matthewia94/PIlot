#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped

import RPi.GPIO as GPIO
import time

#Constants for the robot
wheelRad = .05
robDiam  = .05

#Pins for the motors
MOTORRA = 12
MOTORRB = 16
MOTORLA = 20
MOTORLA = 21
MOTORRE = 24
MOTORRL = 25

#Set velocities to 0 when there haven't been any messages 
vr = 0
vl = 0 

#Set directions to 0 initially
dirr1 = 0
dirr2 = 0
dirl1 = 0
dirl2 = 0

def callback(data):
    vr = ((2 * data.linear.x) + (data.angular.y * robDiam)) / (2 * wheelRad)
    vl = ((2 * data.linear.x) - (data.angular.y * robDiam)) / (2 * wheelRad)

    #Set direction of right motor
    if vr < 0:
        dirr1 = 0
        dirr2 = 1
    else:
        dirr1 = 1
        dirr2 = 0
        
    #Set direction of left motor
    if vl < 0:
        dirl1 = 0
        dirl2 = 1
    else:
        dirl1 = 1
        dirl2 = 0 
    
    #Scale values b/w 0 and 100
    vr = abs(100 * vr)
    vl = abs(100 * vl)

    #Set motor speed through GPIO, pulse enable to reach desired speed
    motorR = GPIO.PWM(MOTORRE, vr)
    motorL = GPIO.PWM(MOTORLE, vl) 
    motorR.start(1)
    motorL.start(1)
    
def motors():
    #Initialize GPIO pins
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(MOTORRA, GPIO.OUT)
    GPIO.setup(MOTORRB, GPIO.OUT)
    GPIO.setup(MOTORLA, GPIO.OUT)
    GPIO.setup(MOTORLB, GPIO.OUT)
    GPIO.setup(MOTORRE, GPIO.OUT)
    GPIO.setup(MOTORLE, GPIO.OUT)     

    #Initialize ros
    rospy.init_node('motors')
    rospy.Subscriber("twist", TwistStamped, callback)
    rospy.spin() 
    motorR.stop()
    motorL.stop()
    GPIO.cleanup()
