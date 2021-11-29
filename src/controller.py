#! /usr/bin/env python

from std_msgs.msg import String
import rospy
import roslib
import time
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import sys


# %tensorflow_version 1.14.0

import tensorflow as tf
from tensorflow.keras import models
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model

sess1 = tf.Session()    
graph1 = tf.get_default_graph()
set_session(sess1)


# roslib.load_manifest('2020_competition')


rospy.init_node('topic_publisher')
license_pub = rospy.Publisher('/license_plate', String, queue_size=1)
vel_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)

cur_dir = os.getcwd()
# cnn_path = cur_dir + "/im_NN_scale 25_11-19-2021-06 39"

# cnn_path = "/home/fizzer/ros_ws/src/enph35controller/src/im_NN_scale 25_11-19-2021-06 39"
cnn_path = '/home/fizzer/ros_ws/src/enph35controller/src/scale_25_02_data_pipeline_update'
cnn = models.load_model(cnn_path)

print("NN loaded, path: ", cnn_path)

x_mag = 0.075
z_mag = 0.387
loop = 0

# time.sleep(1)

license_pub.publish('Team,1234,0,WRXT')

# time.sleep(1)


# start_move = Twist()
# start_move.linear.x = x_mag
# start_move.angular.z = z_mag

# stop = Twist()
# stop.linear.x = 0
# stop.angular.z = 0

# vel_pub.publish(start_move)
# time.sleep(0.3)
# vel_pub.publish(stop)
# time.sleep(0.2)



# for i in range(10):
# 	vel_pub.publish(move)
# 	time.sleep(1)

def image_processing(image):
	scale_percent = 25
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	width = int(image.shape[1] * scale_percent / 100)
	height = int(image.shape[0] * scale_percent / 100)
	dim = (width,height)
	scaled = cv2.resize(gray,dim)
	normal = scaled/255
	return normal


def callback_im(data):

	global cnn
	global z_mag
	global x_mag
	global loop 

	loop += 1

	if loop < 300:

		if loop % 20 == 0:
			print("loop number:", loop)


		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
		processed_img = image_processing(cv_image)

		# im_array = np.array(processed_img)
		# X = np.float32(np.expand_dims(im_array,3))


		test_list = list()
		test_list.append(processed_img)
		X = np.float32(np.expand_dims(np.asarray(test_list),3))


		global sess1
		global graph1 
		with graph1.as_default():
   			set_session(sess1)
   			pred = cnn.predict(X)

		i = np.argmax(pred,axis=1)[0]

		if i == 0:
			z = z_mag
		elif i == 1:
			z = 0
		elif i == 2:
			z = -z_mag

		move = Twist()
		move.linear.x = x_mag
		move.angular.z = z

		vel_pub.publish(move)


	else:
		license_pub.publish('Team1,1234,-1,WRXT')
		sys.exit(0)





image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback_im)

while loop < 300:
	rospy.spin()



