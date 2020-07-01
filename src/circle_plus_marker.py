#!/usr/bin/env python

import rospy
import math
import numpy
import tf
import matplotlib.pyplot as plt
from move_husky import MoveHusky
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion, Point
from visual_path_husky.msg import SysStatus

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA, Bool

class MoveToGoal(object):

	def __init__(self):
		self.pub_obj = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.safety_sub = rospy.Subscriber('/safety_controller', SysStatus, self.SafetyCallback)
		self.sub_obj = rospy.Subscriber('/odometry/filtered', Odometry, self.OdomCallback)

		self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
		self.circle_control_publisher = rospy.Publisher('/circle_control', Bool, queue_size=3)

		self.safety_msg = SysStatus()
		self.iz = 0							#trajectory matrix counter
		self.flag_keeper = False
		self.process_begin = False

		self.move_obj = MoveHusky()

		# minimal radius of 1.5m to function!


	def show_marker_in_rviz(self, marker_publisher, vx, vy):

		pontos = Point()
		marker = Marker()

		marker.header.frame_id = "odom"
		marker.type = marker.SPHERE_LIST
		marker.action = marker.ADD

		# marker scale
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1

		# marker color
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0

		# marker orientaiton
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0

		# marker position
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.0

		# marker line points
		marker.points = []
		k=0

		while k < len(vx):
			pontos.x = vx[k]
			pontos.y = vy[k]
			pontos.z = 0.0
			marker.points.append(Point(vx[k],vy[k],0.0))
			k = k+1

		print(marker.points)
	    # Publish the Marker
		self.marker_publisher.publish(marker)


	def PointsInCircumference(self,initial_position):
		# Finds the radius of the circumference
		r = self.safety_msg.obstacle_size + 0.5		# + 1 m for safety
		if r < 2:
			r = 2

		# Finds the linear coeficient theta
		theta = self.safety_msg.initial_position[2]

		if theta < math.pi/2 and theta > -math.pi/2:
			sin_signal = 1
		else:
			sin_signal = -1

		# Finds the initial angle value to calculate semi circumference
		thetai = theta - math.pi

		# Calculates the origin of the circumference
		origin = [r * math.cos(theta) + initial_position[0], r * math.sin(theta) + initial_position[1]]

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


		plt.plot(x,y,'ro')
		#plt.show()

		self.show_marker_in_rviz(self.marker_publisher, x, y)
		return x, y


	def SafetyCallback(self,status_msg):
		self.safety_msg = status_msg

		if status_msg.flag and not self.process_begin:
			rospy.sleep(0.5)
			self.traj_x, self.traj_y = self.PointsInCircumference(self.safety_msg.initial_position)
			self.flag_keeper = status_msg.flag
			self.process_begin = True
			print(self.safety_msg)

		circle_flag = Bool()
		circle_flag = False
		if self.iz < len(self.traj_x) / 2 and self.process_begin:
			circle_flag = True

		self.circle_control_publisher.publish(circle_flag)


	def OdomCallback(self,robot_state):

		if self.safety_msg.flag == False and self.process_begin == False:
			return
		#initial_point e final_point devem ser parametros de acordo com a informacao recebida do lidar

		if self.safety_msg.found_line:
			self.process_begin = False
			self.iz = 0
			return

		if not self.process_begin:
			return

		v_max = 5.0
		w_max = 2.0

		goal_x = self.traj_x[self.iz]
		goal_y = self.traj_y[self.iz]

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

		# calculates proportional linear and angular velocities
		v_ref = kp * p + 0.1
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

		twist_obj = Twist()

		if self.iz < 2 and abs(a) > 0.15:
			v_ref = 0

		if self.iz >= len(self.traj_x) - 1 :
			twist_obj.linear.x = 0.5
			twist_obj.angular.z = 0.0
			print("Bigger iz")

		# publishes velocities
		twist_obj.linear.x = v_ref
		twist_obj.angular.z = w_ref
		self.move_obj.move_robot(twist_obj)

		if p < 0.2 :
			print("Got to goal ") + str(self.iz)
			self.iz = self.iz + 1




def main():
	rospy.init_node('circle_performer')
	goal_obj = MoveToGoal()

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep


if __name__ == '__main__':
	main()
