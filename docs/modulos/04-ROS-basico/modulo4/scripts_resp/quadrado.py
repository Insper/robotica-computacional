#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import numpy as np

""" 
Rode cada linha em um terminal diferente
	roslaunch my_simulation novas_formas.launch
	rosrun modulo4 quadrado.py
"""

class Control():
	def __init__(self):
		super().__init__()

		# Subscribers
		
		# Publishers
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)

		self.cmd_vel_pub.publish(Twist())
	
	def forward(self, distance: float = 0.5, vel = Twist()) -> None:
		vel.linear.x = 0.5
		self.cmd_vel_pub.publish(vel)
		delta_t = distance / vel.linear.x
		rospy.sleep(delta_t)
		self.cmd_vel_pub.publish(Twist())


	def rotate(self, angle: float = np.pi/2, vel = Twist()) -> None:
		vel.angular.z = np.pi/2
		self.cmd_vel_pub.publish(vel)
		delta_t = angle / vel.angular.z
		rospy.sleep(delta_t)


	def control(self):
		'''
		This function is called at least at {self.rate} Hz.
		This function controls the robot.
		'''
		for _ in range(4):
			self.forward()
			self.rotate()

		
def main():
	rospy.init_node('Controler')
	control = Control()
	rospy.sleep(2)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()
