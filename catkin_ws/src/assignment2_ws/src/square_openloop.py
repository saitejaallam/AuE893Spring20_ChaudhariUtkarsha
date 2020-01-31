#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from math import pi

class square_openloop:
	def __init__(self):
		pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
		rospy.sleep(0.2)
		r = rospy.Rate(5) #per sec 
		while not rospy.is_shutdown():
			vel_msg = Twist()
			vel_msg.linear.x = 0.2
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = 0
			for i in range(50):
				pub.publish(vel_msg)
				r.sleep()
			vel_msg = Twist()
			vel_msg.angular.z = pi/4
			for i in range(10): 
				pub.publish(vel_msg)
				r.sleep() 

if __name__ == "__main__":
	rospy.init_node('square_openloop')
	square_openloop() 

	


