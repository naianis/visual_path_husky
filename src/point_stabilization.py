#!/usr/bin/env/python

import rospy
import math
import numpy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def PointsInCircumference(ip,fp,points):
	# Finds the radius of the circumference
	r = math.sqrt((ip[0]-fp[0])**2 + (ip[1]-fp[1])**2)/2
	print(r)

	# Finds the linear coeficient theta
	theta = math.atan2(fp[1]-ip[1],fp[0]-ip[0])
	print(theta)

	# Finds the initial angle value to calculate semi circumference 
	thetai = theta - math.pi
	print(thetai)

	# Calculates the origin of the circumference
	origin = [r * math.cos(theta) + ip[0], r * math.sin(theta) + ip[1]]
	print(origin)

	# Equacao da reta em x: x(t) = r cos(t) + j
	# Equacao da reta em y: y(t) = r sen(t) + k
	x = numpy.zeros(points)
	y = numpy.zeros(points)
	theta_traj = numpy.zeros(points)
	t=0.0
	i=0
	while t < math.pi:
	#for i in range(points):	#deveria mudar a condicao de parada para theta
		x[i] = r * math.cos(t+thetai) + origin[0]
		y[i] = r * math.sin(t+thetai) + origin[1]
		theta_traj[i] = thetai+t
		t = t + 0.1
		i+=1

	print x

	plt.plot(x,y,'ro')
	#plt.show()

	# Talvez precise tratar caso em que tudo eh zero
	# Funciona pros 4 quadrantes, uhuuu
	return x, y, theta_traj


def OdomCallback(robot_state):
	
	#initial_point e final_point devem ser parametros de acordo com a informacao recebida do lidar
	initial_point = [20, 1]
	final_point = [40, 5]
	maxPoints = 50
	traj_x = traj_y = traj_t = numpy.zeros(maxPoints) 

	traj_x, traj_y, traj_t = PointsInCircumference(initial_point,final_point,maxPoints)

	k01, k02, k03 = 2, 0.2, 20
	v_nav = 0.5
	w_nav = 0.5

	iz = 1

	x_ref = traj_x[iz]
	y_ref = traj_y[iz]
	t_ref = traj_t[iz]

	e1 = math.cos(robot_state.pose.pose.orientation.z) * (x_ref - robot_state.pose.pose.position.x) + math.sin(robot_state.pose.pose.orientation.z) * (y_ref - robot_state.pose.pose.position.y)
	e2 = -math.sin(robot_state.pose.pose.orientation.z)*(x_ref - robot_state.pose.pose.position.x) + math.cos(robot_state.pose.pose.orientation.z)*(y_ref - robot_state.pose.pose.position.y)
	e3 = t_ref - robot_state.pose.pose.orientation.w

	v_ref = v_nav * math.cos(e3) + k01 * e1
	w_ref = w_nav + k02 * v_nav * e2 + k03 * v_nav * math.sin(e3) 

	pub_obj = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

	twist_obj = Twist()
	twist_obj.linear.x = v_ref
	twist_obj.angular.z = w_ref
	pub_obj.publish(twist_obj)

	if abs(e1-e2)<=0.1 and abs(e3)<=0.1:
		iz+=1
		
		if(iz>=50):
			twist_obj.linear.x = 0.0
			twist_obj.angular.z = 0.0
			pub_obj.publish(twist_obj)
			rospy.sleep(1)


def main():
	rospy.init_node('point_stabilization')
	rate = rospy.Rate(10)
	#ctrl_c stuff here!!

	sub_obj = rospy.Subscriber('/odometry/filtered', Odometry, OdomCallback)
	rospy.spin()

if __name__ == '__main__':
	main()
