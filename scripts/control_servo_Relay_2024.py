
#!usr/bin/env
import rospy
from std_msgs.msg import Float64
from drone_control.msg import send_value
#from gpiozero import Servo
#from time import sleep

def callback(data):
    x=data.accept
    #rospy.loginfo("%f",data.accept)
    #servo=Servo(18)
    if x==1:
        rospy.loginfo("here: %s", data.data)
        #servo.min()
        #sleep(2)
        #servo.mid()
        #sleep(2)
        #servo.max()
        #sleep(2)
    
def listener():
    rospy.init_node("Sub_node",anonymous=True)
    rospy.Subscriber('send_value',send_value,callback)
    rospy.spin()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInitException:
        pass
	
