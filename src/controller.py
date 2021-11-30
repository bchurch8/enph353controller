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
# cnn_path = '/home/fizzer/ros_ws/src/enph35controller/src/scale_25_04_data_pipeline_update' #sometimes
 # makes it around first corner and into second corner also sometimes crashes 
# cnn_path = "/home/fizzer/ros_ws/src/enph35controller/src/scale_25_new_driving_meth_01"
# cnn_path = "/home/fizzer/ros_ws/src/enph35controller/src/02_L 1259,S 1500" #Consistently makes it around 
# first corner but turns into inner circle
# cnn_name = "/02_croppedL 590,S 1335" #always turns into the Cars
# cnn_name = "/03_croppedL 877,S 1335" #Consistently makes it around 
# first corner but turns into inner circle
# cnn_name = "/04_croppedL 971,S 1940" #Seems to turn fine on corners without cross walks
#and turns early on corners with cross walks
# cnn_name = "/05_croppedL 1063,S 2498" # always goes straight -> too much S data
# cnn_name = "/06_croppedL 1063,S 2000" #MAKES IT AROUND 2ND CORNER SOMETIMES
cnn_name = "/07_croppedL 1063,S 1750" #Corners decently well but still randomly turns into other features
cnn_path = cur_dir + cnn_name

cnn = models.load_model(cnn_path)

print("NN loaded, path: ", cnn_path)

x_mag = 0.075
z_mag = 0.387
loop = 0
loop_count = 600

# time.sleep(1)

license_pub.publish('Team,1234,0,WRXT')

# time.sleep(1)


start_move = Twist()
start_move.linear.x = x_mag
start_move.angular.z = 0

vel_pub.publish(start_move)
time.sleep(0.5)

start_move.linear.x = 0
start_move.angular.z = z_mag
vel_pub.publish(start_move)
time.sleep(0.5)

stop = Twist()
stop.linear.x = 0
stop.angular.z = 0


vel_pub.publish(stop)


# def image_processing(image):
# 	scale_percent = 25
# 	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# 	width = int(image.shape[1] * scale_percent / 100)
# 	height = int(image.shape[0] * scale_percent / 100)
# 	dim = (width,height)
# 	scaled = cv2.resize(gray,dim)
# 	normal = scaled/255
# 	return normal

def image_processing(image):

	scale_percent = 25

	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	height = gray.shape[0]
	cropped = gray[int(height/2):height,:]

	scale_width = int(cropped.shape[1] * scale_percent / 100)
	scale_height = int(cropped.shape[0] * scale_percent / 100)
	dim = (scale_width, scale_height)

	scaled = cv2.resize(cropped,dim)
	normal = scaled/255

	return normal


def callback_im(data):

	global cnn
	global z_mag
	global x_mag
	global loop 

	loop += 1

	if loop < loop_count:

		# if loop % 20 == 0:
		# 	print("loop number:", loop)


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
			x = 0
			z = z_mag
		elif i == 1:
			x = x_mag
			z = 0
		elif i == 2:
			z = -z_mag
			x = 0

		move = Twist()
		move.linear.x = x
		move.angular.z = z

		vel_pub.publish(move)


	if loop == loop_count:
		license_pub.publish('Team1,1234,-1,WRXT')
		loop +=1





image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback_im)

while loop < loop_count:
	rospy.spin()



