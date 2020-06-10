#!/usr/bin/env python

import rospy
import math
import numpy
import tf
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion

class MoveToGoal(object):

	def __init__(self):
		self.sub_obj = rospy.Subscriber('/odometry/filtered', Odometry, self.OdomCallback)
		self.pub_obj = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

		self.robot_state = Odometry()

	def OdomCallback(self,robot_state):

		#initial_point e final_point devem ser parametros de acordo com a informacao recebida do lidar


		goal_x = 3
		goal_y = 5

		quaternion = (robot_state.pose.pose.orientation.x, robot_state.pose.pose.orientation.y, robot_state.pose.pose.orientation.z, robot_state.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw = euler[2]

		dx = goal_x - robot_state.pose.pose.position.x
		dy = goal_y - robot_state.pose.pose.position.y

		kp, ka = 0.2, 0.5

		p = math.sqrt( pow(dx,2) + pow(dy,2) )
		a = -yaw + math.atan2(dy,dx)

		print ("p = ") + str(p) + (" a = ") + str(a)

		v_ref = kp * p + 0.1
		w_ref = ka * a

		print ("robot_state_x ") + str(robot_state.pose.pose.position.x) +(" robot_state_y ") + str(robot_state.pose.pose.position.y) +(" robot_state_theta ") + str(yaw)
		print ("velocidade v ") + str(v_ref) +(" velocidade w ") + str(w_ref)
		print(" ")


		twist_obj = Twist()
		twist_obj.linear.x = v_ref
		twist_obj.angular.z = w_ref
		self.pub_obj.publish(twist_obj)

		if p < 0.1 :
			print("GOT TO GOAL") + str(robot_state)

			twist_obj = Twist()
			twist_obj.linear.x = 0.0
			twist_obj.angular.z = 0.0
			self.pub_obj.publish(twist_obj)
			rospy.loginfo("Shutdown time!")
			rospy.sleep(1)
			rospy.signal_shutdown("an exception")



def main():
	rospy.init_node('point_stabilization')
	goal_obj = MoveToGoal()

	rate = rospy.Rate(5)

	while not rospy.is_shutdown():
		rate.sleep


if __name__ == '__main__':
	main()
