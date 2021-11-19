#! /usr/bin/env python


import rospy
import roslib
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import time
from datetime import datetime

from geometry_msgs.msg import Twist

# rate = rospy.Rate(2)
rospy.init_node('topic_publisher')

directory = '/home/fizzer/ros_ws/src/comp_controller/src/labelled_driving_data'
x = 0
z = 0
i = 0

cur_date = datetime.now()
cur_date_str = cur_date.strftime('%m-%d-%Y-%H:%M')
print("Ready to record driving data")


def callback_im(data):

	if x != 0:
		global i
		global directory
		global cur_date_str

		im_num = str(i)

		im_num_len = len(im_num)

		if im_num_len == 1:
			im_num = "000" + im_num
		elif im_num_len == 2:
			im_num = "00" + im_num
		elif im_num_len == 3:
			im_num = "0" + im_num


		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

		os.chdir(directory)

		im_name = cur_date_str + '_' + im_num + '_x:' + str(x) + ',z:' + str(z) + '.png'

		cv2.imwrite(im_name, cv_image)
		i += 1


def callback_vel(data):
	global x
	global z
	cmd_vel = data
	x = cmd_vel.linear.x
	z = cmd_vel.angular.z


image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback_im)
vel_sub = rospy.Subscriber('/R1/cmd_vel', Twist, callback_vel)

while not rospy.is_shutdown():
    rospy.spin()
