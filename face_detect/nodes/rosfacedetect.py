#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import roslib
roslib.load_manifest('face_detect_base')

import sys
import os

import rospy
import sensor_msgs.msg
from face_detect_base.msg import RoiArray # Custom msg
from std_msgs.msg import Float64
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import *


min_size = (10, 10)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0
display = True
L = 0.33 
R = 0.10

if __name__ == '__main__':
    opencv_dir = '/usr/share/opencv/haarcascades/';

    face_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_frontalface_default.xml')
    if face_cascade.empty():
        print "Could not find face cascade"
        sys.exit(-1)
    eye_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_eye.xml')
    if eye_cascade.empty():
        print "Could not find eye cascade"
        sys.exit(-1)
    br = CvBridge()
    rospy.init_node('facedetect')
    display = rospy.get_param("~display",True)

    def detect_and_draw(imgmsg):
        """
        Process an image and draw frames arounf detected faces and eyes in the image/

        Args:
            imgmsg: Image posted on the Image Topic
        """
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        # allocate temporary images
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces_array = []
        faces = face_cascade.detectMultiScale(gray, 1.3, 3)
        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            eyes = eye_cascade.detectMultiScale(roi_gray)
            # ROI Object to send to ROS topic
            ROI = sensor_msgs.msg.RegionOfInterest()
            ROI.x_offset = x
            ROI.y_offset = y
            ROI.width = w
            ROI.height = h
            ROI.do_rectify = False
            # Adding it to an array, in case of multiple detections
            faces_array.append(ROI)
            pub.publish(ROI)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        
        cv2.imshow('img',img)
        cv2.waitKey(10)
        custom_msg = RoiArray()
        custom_msg.roi_array = faces_array
        pub2.publish(custom_msg)

    def pilote(data):
        """
        Send the Joystick commands to the robot

        Args:
            data: Command from the joystick to convert
        """
        wl_value = (data.axes[1] - data.axes[0]*L/2)/R
        wr_value = (data.axes[1] + data.axes[0]*L/2)/R
        wl.publish(Float64(wl_value))
        wr.publish(Float64(wr_value))


    rospy.Subscriber("~image", sensor_msgs.msg.Image, detect_and_draw)
    # Publisher of only one Region of Interest (ROI)
    pub = rospy.Publisher("myROI", sensor_msgs.msg.RegionOfInterest, 1000)
    # Publisher of an array of ROIs
    pub2 = rospy.Publisher("myROI2", RoiArray, 1000)
    # Listener to the Joystick topic
    rospy.Subscriber('joy', Joy, pilote)
    # Left and right wheel publisher
    wl = rospy.Publisher('/vrep/leftWheelCommand', Float64, 1)    
    wr = rospy.Publisher('/vrep/rightWheelCommand', Float64, 1)
    rospy.spin()
