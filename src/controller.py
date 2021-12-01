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
# cnn_name = "/08_croppedL 1391,S 2499" #Makes it all the way around sometimes!!

cnn_path = cur_dir + cnn_name

cnn = models.load_model(cnn_path)

print("NN loaded, path: ", cnn_path)

def set_move(lin_x,ang_z):
	move = Twist()
	move.linear.x = lin_x
	move.angular.z = ang_z

	return move

#Global Variables

x_mag = 0.15
#initial was 0.075 x and 0.30 z
z_mag = 0.30

cross_time = time.time()
cross_wait = False

start_time = rospy.get_time()
lap_complete = False
lap_time = 111

straight = set_move(x_mag, 0)
left = set_move(0, z_mag)
right = set_move(0, -z_mag)
stop = set_move(0,0)
crossing = False

#Function Definitions

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

def sidewalk_thresh(image):
	# Convert the RGB image to HSV
	image_HSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

	#HSV filter to identify the walk
	uh = 0
	us = 255
	uv = 255
	lh = 0
	ls = 30
	lv = 150
	lower_hsv = np.array([lh,ls,lv])
	upper_hsv = np.array([uh,us,uv])

	# Threshold the HSV image to  only get the sidewalk
	walk_mask = cv2.inRange(image_HSV, lower_hsv, upper_hsv)

	kernel2 = 255 * np.ones((5,5),np.uint8)
	dilation = cv2.dilate(walk_mask, kernel2, iterations = 1)
	return dilation

def pedestrian_thresh(image):

  image_HSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

  uh = 152
  us = 255
  uv = 117
  lh = 86
  ls = 45
  lv = 0
  lower_hsv = np.array([lh,ls,lv])
  upper_hsv = np.array([uh,us,uv])

  pedestrian_thresh = cv2.inRange(image_HSV, lower_hsv, upper_hsv)
  kernel = 255 * np.ones((5,5),np.uint8)
  opening = cv2.morphologyEx(pedestrian_thresh, cv2.MORPH_OPEN, kernel)
  kernel2 = 255 * np.ones((6,6),np.uint8)
  dilation = cv2.dilate(opening, kernel2, iterations = 1)

  return dilation
	

def sidewalk_detector(thresh_im):
	im, contours, hierarchy = cv2.findContours(thresh_im,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	if len(contours) < 1:
		return False

	x,y,w,h = cv2.boundingRect(contours[0])

	if (thresh_im.shape[0] - (y+h)) < 60 and w*h > 50000:
		return True
	
	else:
		return False

def ped_detector(top_cross_thresh):
  im, cross_contours, cross_hierarchy = cv2.findContours(top_cross_thresh,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

  if(len(cross_contours) >= 2):
    return True
    
  return False

def callback_im(data):

	global cnn
	global stop
	global straight
	global left
	global right
	global cross_time
	global cross_wait
	global start_time
	global lap_complete

	if not lap_complete:

		if(rospy.get_time() - start_time > lap_time):
			license_pub.publish('Team1,1234,-1,WRXT')
			lap_complete = True
			vel_pub.publish(stop)
		else:
			bridge = CvBridge()
			cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

			height = cv_image.shape[0]
			third = int (height /3)

			bot_cropped = cv_image[2*third:height,:]
			bot_thresh = sidewalk_thresh(bot_cropped)
			sidewalk_pres = sidewalk_detector(bot_thresh)

			if cross_wait:
				top_cropped = cv_image[third:2*third,:]
				cross_thresh = sidewalk_thresh(top_cropped)
				ped_pres = ped_detector(cross_thresh)

				if(ped_pres):
					print("Pedestrian detected")
					rospy.sleep(0.5)
					vel_pub.publish(set_move(2*x_mag,0))
					rospy.sleep(1)
					vel_pub.publish(stop)
					cross_wait = False
				elif((time.time() - cross_time) > 4):
					print("Time elapsed, moving forward")
					rospy.sleep(1)
					vel_pub.publish(set_move(3*x_mag,0))
					rospy.sleep(1)
					vel_pub.publish(stop)
					cross_wait = False
			elif (sidewalk_pres and (time.time() - cross_time > 8)):
				print("Sidewalk detected")
				cross_time = time.time()
				cross_wait = True
				vel_pub.publish(stop)


			else:
				processed_img = image_processing(cv_image)

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
					move = left
					# x = 0
					# z = z_mag
				elif i == 1:
					move = straight
				elif i == 2:
					move = right

				vel_pub.publish(move)


license_pub.publish('Team,1234,0,WRXT')

vel_pub.publish(straight)
rospy.sleep(0.5)

vel_pub.publish(left)
rospy.sleep(0.5)
vel_pub.publish(stop)

image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback_im, queue_size = 1)


while True:
	rospy.spin()



