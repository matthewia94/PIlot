#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped

import RPi.GPIO as GPIO
import time

#Constants for the robot
wheelRad = 3.5
robDiam  = 10.0

#Pins for the motors
MOTORRA = 12
MOTORRB = 16
MOTORLA = 20
MOTORLB = 21
MOTORRE = 24
MOTORLE = 25

#Initialize GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTORRA, GPIO.OUT)
GPIO.setup(MOTORRB, GPIO.OUT)
GPIO.setup(MOTORLA, GPIO.OUT)
GPIO.setup(MOTORLB, GPIO.OUT) 
GPIO.setup(MOTORRE, GPIO.OUT)
GPIO.setup(MOTORLE, GPIO.OUT)     
motorR = GPIO.PWM(MOTORRE, 50)
motorL = GPIO.PWM(MOTORLE, 50)
motorR.start(0)
motorL.start(0)

#Set velocities to 0 when there haven't been any messages 
vr = 0
vl = 0 

#Set directions to 0 initially
dirr1 = 0
dirr2 = 0
dirl1 = 0
dirl2 = 0

def callback(data):
    vr = ((2 * data.twist.linear.x) + (data.twist.angular.y * robDiam)) / (2 * wheelRad)
    vl = ((2 * data.twist.linear.x) - (data.twist.angular.y * robDiam)) / (2 * wheelRad)

    #Set direction of right motor
    if vr < 0:
        dirr1 = GPIO.LOW
        dirr2 = GPIO.HIGH
    else:
        dirr1 = GPIO.HIGH 
        dirr2 = GPIO.LOW 
        
    #Set direction of left motor
    if vl < 0:
        dirl1 = GPIO.LOW
        dirl2 = GPIO.HIGH
    else:
        dirl1 = GPIO.HIGH
        dirl2 = GPIO.LOW 
    
    #Scale values b/w 0 and 100
    vr = abs((100 * vr) / ((2 + robDiam) / (2 * 3.5)))
    vl = abs((100 * vl) / ((2 + robDiam) / (2 * 3.5)))

    #Set motor speed through GPIO, pulse enable to reach desired speed
    motorR.ChangeDutyCycle(vr)
    motorL.ChangeDutyCycle(vl)

    #Set up the direction of the motors
    GPIO.output(MOTORRA, dirr1)
    GPIO.output(MOTORRB, dirr2)
    GPIO.output(MOTORLA, dirl1)
    GPIO.output(MOTORLB, dirl2)
    
def motors():
    #Initialize ros
    rospy.init_node('motors')
    rospy.Subscriber("twist", TwistStamped, callback)
    rospy.spin() 
    motorR.stop()
    motorL.stop()
    GPIO.cleanup()

if __name__ == '__main__':
	motors();
