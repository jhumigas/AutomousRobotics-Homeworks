#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

L = 0.33
R = 0.10
def callback1(data):
    wl = (data.axes[1] - data.axes[0]*L/2)/R
    pub1.publish(Float64(wl))

def callback2(data):
    wr = (data.axes[1] + data.axes[0]*L/2)/R
    pub2.publish(Float64(wr))

def callback(data):
    callback1(data)
    callback2(data)

def start():
    global pub1
    global pub2
    pub1 = rospy.Publisher('pioneer_p3dx/leftWheelCommand', Float64)
    pub2 = rospy.Publisher('pioneer_p3dx/rightWheelCommand', Float64)
    rospy.Subscriber('joy', Joy, callback)
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()    
