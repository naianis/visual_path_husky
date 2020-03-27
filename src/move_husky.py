#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class MoveHusky(object):

	def __init__(self):
		self.pub_obj = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		self.last_cmdvel_command = Twist()
		self.cmdvel_pubrate = rospy.Rate(10)
		self.shutdown_detected = False

	def move_robot(self, twist_obj):
		self.pub_obj.publish(twist_obj)

	def clean_class(self):
		stop_twist_obj = Twist()
		stop_twist_obj.angular.z = 0.0
		stop_twist_obj.linear.x = 0.0
		self.move_robot(stop_twist_obj)
		self.shutdown_detected = True



def main():
	rospy.init_node('move_husky', anonymous=True)

	
	move_obj = MoveHusky()
	twist_obj = Twist()

	twist_obj.angular.z = 0.5
	rate = rospy.Rate(5)
	ctrl_c = False

	def shutdownhook():
		move_husky.clean_class()
		rospy.loginfo("Shutdown time!")
		ctrl_c = True

	while not ctrl_c:
		move_obj.move_robot(twist_obj)
		rate.Sleep()


if __name__ == '__main__':
	main()