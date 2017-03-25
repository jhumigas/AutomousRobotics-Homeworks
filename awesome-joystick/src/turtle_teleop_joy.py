#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    """
    Broadcast joystick data to the turtle
    Since the turtle is expecting twist, we convert those data to twists
    Once the twist is ready, it publush to the turtle's topic

    Args: 
        data(dict): Values sent by the joy stick
    """
    twist = Twist()
    twist.linear.x = 5*data.axes[1]
    twist.angular.z = 4*data.axes[0]
    pub.publish(twist)

def start():
    """
    Initialize publisher and subscriber
    """
    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist)
    rospy.Subscriber('joy', Joy, callback)
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()
