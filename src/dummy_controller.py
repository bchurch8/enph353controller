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


move = Twist()
move.linear.x = 0.3
move.angular.z = 0.6

for i in range(10):
	vel_pub.publish(move)
	time.sleep(1)

license_pub.publish('Team1,1234,-1,WRXT')

print("Controller script finished")
