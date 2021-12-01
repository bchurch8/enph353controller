#! /usr/bin/env python

from std_msgs.msg import String
import rospy
import roslib
import time
from geometry_msgs.msg import Twist

# roslib.load_manifest('2020_competition')


rospy.init_node('topic_publisher')
license_pub = rospy.Publisher('/license_plate', String, queue_size=1)
vel_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)

print("Beginning controller script")

time.sleep(1)

license_pub.publish('Team,1234,0,WRXT')

time.sleep(1)

x_mag = 0.075
z_mag = 0.30

start_straight = Twist()
start_straight.linear.x = x_mag
start_straight.angular.z = 0

stop = Twist()
stop.linear.x = 0
stop.angular.z = 0


vel_pub.publish(start_straight)
time.sleep(2)
vel_pub.publish(stop)

# for i in range(10):
# 	vel_pub.publish(move)
# 	time.sleep(1)

license_pub.publish('Team1,1234,-1,WRXT')

print("Controller script finished")
