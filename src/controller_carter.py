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
# cnn_name = "/07_croppedL 1063,S 1750" #Corners decently well but still randomly turns into other features
#^best so far
cnn_name = "/08_croppedL 1391,S 2499"

cnn_path = cur_dir + cnn_name

cnn = models.load_model(cnn_path)

print("NN loaded, path: ", cnn_path)

x_mag = 0.075
z_mag = 0.30
loop = 0
loop_count = 600
cross_time = time.time()
plate_time = time.time()


# time.sleep(1)

license_pub.publish('Team,1234,0,WRXT')

# time.sleep(1)

def set_move(lin_x,ang_z):
	move = Twist()
	move.linear.x = lin_x
	move.angular.z = ang_z

	return move

straight = set_move(x_mag, 0)
left = set_move(0, z_mag)
right = set_move(0, -z_mag)
stop = set_move(0,0)
crossing = False

vel_pub.publish(straight)
time.sleep(0.5)

vel_pub.publish(left)
time.sleep(0.5)


vel_pub.publish(stop)

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

def car_thresh(image):
	height = image.shape[0]
	width = image.shape[1]
	im_crop = image[int(height/2):height,0:int(width/2)]

	# cv2.imshow('',im_crop)
	# cv2.waitKey(1)

# Crop the image to include only bottom left section
	# im_crop = image[300:720, 0:600]

  # Convert the RGB image to HSV
	image_HSV = cv2.cvtColor(im_crop, cv2.COLOR_RGB2HSV)

    #HSV filter to identify car
	uh = 141
	us = 255
	uv = 218
	lh = 92
	ls = 100
	lv = 90

	lower_hsv = np.array([lh,ls,lv])
  	upper_hsv = np.array([uh,us,uv])

	# Threshold the HSV image to  only get the car
	car_mask = cv2.inRange(image_HSV, lower_hsv, upper_hsv)
	# cv2.imshow('',car_mask)
	# cv2.waitKey(1)

	number_of_white_pix = np.sum(car_mask == 255)
  	print(number_of_white_pix)
	return (number_of_white_pix,im_crop)

def cropped_plate(image):
	# Convert the RGB image to HSV
	image_HSV = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

	#HSV filter to identify plate
	uh = 170
	us = 200
	uv = 194
	lh = 100
	ls = 0
	lv = 50
	lower_hsv = np.array([lh,ls,lv])
	upper_hsv = np.array([uh,us,uv])

	# Threshold the HSV image to get only the plate
	plate_mask = cv2.inRange(image_HSV, lower_hsv, upper_hsv)

	# Prime plate for contouring
	kernel = 255 * np.ones((4,4),np.uint8)
	erosion = cv2.erode(plate_mask,kernel,iterations = 1)
	dilation = cv2.dilate(erosion, kernel, iterations = 2)
	opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)

	# Get Contour
	im, contours, hierarchy = cv2.findContours(opening,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


	# Get Plate Location
	x,y,w,h = cv2.boundingRect(contours[0])

	# Hard coded -5 to get nicer crop
	plate = image[y:y+h-5, x:x+w-5]

	return plate

def sidewalk_thresh(image):
	# Convert the BRG image to HSV
	height = image.shape[0]
	cropped = image[int(2*height/3):height,:]
	
	image_HSV = cv2.cvtColor(cropped, cv2.COLOR_RGB2HSV)

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
	# cv2.imshow('', walk_mask)
	# cv2.waitKey(1)
	return walk_mask

def sidewalk_detector(thresh_im):
	im, contours, hierarchy = cv2.findContours(thresh_im,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	if len(contours) < 1:
		return False

	x,y,w,h = cv2.boundingRect(contours[0])

	if (thresh_im.shape[0] - (y+h)) < 60 and w*h > 50000:
	# if (thresh_im.shape[0] - (y+h)) < 60 :
		# print(thresh_im.shape[0] - (y+h))
		# print(w*h)
		return True
	
	else:
		return False

def callback_im(data):

	global cnn
	global loop 
	global stop
	global straight
	global left
	global right
	global cross_time
	global plate_time


	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	thresh = sidewalk_thresh(cv_image)
	sidewalk_pres = sidewalk_detector(thresh)

	whitepix,crop_car = car_thresh(cv_image)


	if (whitepix>27000 and (time.time() - plate_time > 8)):
		plate_time = time.time()
		print("Plate detected")
		plate = cropped_plate(crop_car)
		cv2.imshow('',plate)
		cv2.waitKey(1)
	



	if(sidewalk_pres and (time.time() - cross_time > 4)):
		print("Sidewalk detected")
		cross_time = time.time()
		
		vel_pub.publish(stop)
		rospy.sleep(1)
		vel_pub.publish(set_move(2*x_mag, 0))
		rospy.sleep(3)
		vel_pub.publish(stop)

		


	else:

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
			move = left
			# x = 0
			# z = z_mag
		elif i == 1:
			move = straight
			# x = x_mag
			# z = 0
		elif i == 2:
			move = right
			# z = -z_mag
			# x = 0

		# move = Twist()
		# move.linear.x = x
		# move.angular.z = z

		vel_pub.publish(move)


		# if loop == loop_count:
		# 	license_pub.publish('Team1,1234,-1,WRXT')
		





image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback_im, queue_size = 1)

while loop < loop_count:
	rospy.spin()



