#! /usr/bin/env python


import rospy
import roslib
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import time

from geometry_msgs.msg import Twist

# rate = rospy.Rate(2)
rospy.init_node('topic_publisher')

directory = '/home/fizzer/ros_ws/src/comp_controller/src/labelled_driving_data'
x = 0
z = 0



def callback_im(data):
	# global i
	

	# if(i == 10):
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

	# 	os.chdir(directory)

	# 	cv2.imwrite("test.png", cv_image)

	# 	i = 0

	# else:
	# 	i += 1
	global current_im
	current_im = cv_image


def callback_vel(data):
	global cmd_vel
	cmd_vel = data
	x = cmd_vel.linear.x
	z = cmd_vel.angular.z

	global directory
	global current_im


	os.chdir(directory)

	im_name = 'x:' + str(x) + ',z:' + str(z) + '.png'

	cv2.imwrite(im_name, current_im)


image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback_im)
vel_sub = rospy.Subscriber('/R1/cmd_vel', Twist, callback_vel)

while not rospy.is_shutdown():
    rospy.spin()
