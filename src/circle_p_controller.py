#!/usr/bin/env python

import rospy
import math
import numpy
import tf
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from visual_path_husky.msg import SysStatus

class MoveToGoal(object):

	def __init__(self):
		self.pub_obj = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.safety_sub = rospy.Subscriber('/safety_controller', SysStatus, self.SafetyCallback)
		self.sub_obj = rospy.Subscriber('/odometry/filtered', Odometry, self.OdomCallback)


		self.safety_msg = SysStatus()
		self.iz = 0							#trajectory matrix counter
		self.flag_keeper = False
		self.process_begin = False

		# minimal radius of 1.5m to function!



	def PointsInCircumference(self,initial_position):
		# Finds the radius of the circumference
		#r = math.sqrt((ip[0]-fp[0])**2 + (ip[1]-fp[1])**2)/2
		r = self.safety_msg.obstacle_size + 1		# + 1 m for safety
		if r < 2:
			r = 2
		print("radius = ") + str(r)

		# Finds the linear coeficient theta
		#theta = math.atan2(fp[1]-ip[1],fp[0]-ip[0])
		theta = self.safety_msg.initial_position[2]
		print("theta = ") + str(theta)

		if theta < math.pi/2 and theta > -math.pi/2:
			sin_signal = 1
		else:
			sin_signal = -1
		print("sin signal = ") + str(sin_signal)

		# Finds the initial angle value to calculate semi circumference
		thetai = theta - math.pi
		#print(thetai)

		# Calculates the origin of the circumference
		origin = [r * math.cos(theta) + initial_position[0], r * math.sin(theta) + initial_position[1]]
		#print(origin)

		# Equacao da reta em x: x(t) = r cos(t) + j
		# Equacao da reta em y: y(t) = r sen(t) + k
		x = numpy.zeros(33)
		y = numpy.zeros(33)
		t=0.0
		i=0

		# calculates the circle matrix
		while t < math.pi:
			x[i] = r * math.cos(t+thetai) + origin[0]
			y[i] = r * math.sin(t+thetai) * sin_signal + origin[1]
			t = t + 0.1
			i+=1

		print(x)
		print(y)
		plt.plot(x,y,'ro')
		#plt.show()

		return x, y


	def SafetyCallback(self,status_msg):
		self.safety_msg = status_msg

		if self.flag_keeper != status_msg.flag and self.process_begin == False:
			self.traj_x, self.traj_y = self.PointsInCircumference(self.safety_msg.initial_position)
			self.flag_keeper = status_msg.flag
			self.process_begin = True
			print(self.safety_msg)


	def OdomCallback(self,robot_state):

		if self.safety_msg.flag == False and self.process_begin == False:
			return
		#initial_point e final_point devem ser parametros de acordo com a informacao recebida do lidar

		v_max = 5.0
		w_max = 1.0

		goal_x = self.traj_x[self.iz]
		goal_y = self.traj_y[self.iz]
		print ("goal x = ") + str(goal_x) + (" goal y = ") + str(goal_y) + ("    iz = ") + str(self.iz)

		# transform from quaternion for yaw degrees
		quaternion = (robot_state.pose.pose.orientation.x, robot_state.pose.pose.orientation.y, robot_state.pose.pose.orientation.z, robot_state.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw = euler[2]

		dx = goal_x - robot_state.pose.pose.position.x
		dy = goal_y - robot_state.pose.pose.position.y

		# velocities coeficients
		kp = 0.1
		ka = 1

		# linear and angular errors
		p = math.sqrt( pow(dx,2) + pow(dy,2) )
		a = -yaw + math.atan2(dy,dx)

		print ("p = ") + str(p) + (" a = ") + str(a)

		# calculates proportional linear and angular velocities
		v_ref = kp * p + 0.2
		w_ref = ka * a

		#restricts the maximum velocities
		if v_ref > v_max:
			v_ref = v_max
		if v_ref < -v_max:
			v_ref = - v_max

		if w_ref > w_max:
			w_ref = w_max
		if w_ref < -w_max:
			w_ref = -w_max

		print ("robot_state_x ") + str(robot_state.pose.pose.position.x) +(" robot_state_y ") + str(robot_state.pose.pose.position.y) +(" robot_state_theta ") + str(yaw)
		print ("velocidade v ") + str(v_ref) +(" velocidade w ") + str(w_ref)
		print(" ")

		# publishes velocities
		twist_obj = Twist()
		twist_obj.linear.x = v_ref
		twist_obj.angular.z = w_ref
		self.pub_obj.publish(twist_obj)

		if p < 0.2 :
			print("GOT TO GOAL") + str(robot_state)
			self.iz = self.iz + 1

			if self.iz >=32 :
				twist_obj = Twist()
				twist_obj.linear.x = 0.0
				twist_obj.angular.z = 0.0
				self.pub_obj.publish(twist_obj)
				process_begin = False
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
