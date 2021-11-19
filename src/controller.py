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


# %tensorflow_version 1.14.0

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers

from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend
from tensorflow.keras import models

# roslib.load_manifest('2020_competition')


rospy.init_node('topic_publisher')
license_pub = rospy.Publisher('/license_plate', String, queue_size=1)
vel_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)

cnn_path = "/home/fizzer/ros_ws/src/comp_controller/src/imitation_learning_model"

cnn = models.load_model(cnn_path)\

print("NN loaded")

x_mag = 0.075
z_mag = 0.387

time.sleep(1)

license_pub.publish('Team,1234,0,WRXT')

time.sleep(1)


# move = Twist()
# move.linear.x = 0.3
# move.angular.z = 0.6

# for i in range(10):
# 	vel_pub.publish(move)
# 	time.sleep(1)

def image_processing(image):
	scale_percent = 25
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	width = int(img.shape[1] * scale_percent / 100)
	height = int(img.shape[0] * scale_percent / 100)
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

	if loop < 1000:

		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
		processed_img = image_processing(cv_image)


		test_list = list()
		test_list.append(processed_img)
		X = np.float32(np.expand_dims(np.asarray(test_list),3))
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





image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, callback_im)

while loop < 1000:
	rospy.spin()



