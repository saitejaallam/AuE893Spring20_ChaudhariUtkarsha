#!/usr/bin/env python
#chaudhri_utkarsha

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        # self.pose.theta = round(self.pose.theta, 4)
        # print('self theta -',abs(self.pose.theta))

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def angular_difference(self,angle):
        return (angle - self.pose.theta)

    def rotate(self,angle,constant=1.5):
        return constant* (self.angular_difference(angle))

    def move2goal(self):
	"""Moves the turtle to the goal."""
        goal_pose = Pose()

        # Giving inputs
        x = [0,5,8,8,5,5]
        y = [0,5,5,8,8,5]
        theta = [-3*pi/4,0,pi/2,pi,-pi/2,0]

        distance_tolerance = 0.01
        angle_tolerance = 0.01

        vel_msg = Twist()

        for i in range(6):
            goal_pose.x = x[i]
            goal_pose.y = y[i]

            while self.euclidean_distance(goal_pose) >= distance_tolerance:
                vel_msg.linear.x = self.linear_vel(goal_pose)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel(goal_pose)

		# Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

		# Publish at the desired rate.
                self.rate.sleep()

            while abs(self.angular_difference(theta[i])) >= angle_tolerance:
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.rotate(theta[i])

		# Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

		# Publish at the desired rate.
                self.rate.sleep()

	# Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
