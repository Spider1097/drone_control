#!/usr/bin/env python

import rospy
from drone_control.msg import send_value
import RPi.GPIO as GPIO
from time import sleep

def callback(data):
    rospy.loginfo("Received accept value: %f", data.accept)
    move_servo_based_on_accept(data.accept)

def move_servo_based_on_accept(accept_value):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.OUT)
    p = GPIO.PWM(11, 50)
    p.start(0)
    if accept_value == 0:
        p.ChangeDutyCycle(7.5)   
    elif accept_value == 1:
        p.ChangeDutyCycle(3)     
    elif accept_value == 2:
        p.ChangeDutyCycle(12.5)  
    elif accept_value == 3:
        p.ChangeDutyCycle(9.5)   
    sleep(0.5)                   
    p.stop()                    
    GPIO.cleanup()               

def listener():
    rospy.init_node('listener2', anonymous=True)
    rospy.Subscriber('/send_value', send_value, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()