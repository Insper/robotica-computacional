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
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)
		self.dormir = 5.0

		self.cmd_vel_pub.publish(Twist())
	
	def forward(self, distance: float = 0.5, vel = Twist()) -> None:
		vel.linear.x = distance / self.dormir
		self.cmd_vel_pub.publish(vel)
		rospy.sleep(self.dormir)

	def rotate(self, angle: float = np.pi/2, vel = Twist()) -> None:
		vel.angular.z = angle / self.dormir
		self.cmd_vel_pub.publish(vel)
		rospy.sleep(self.dormir)

	def control(self):
		'''
		This function is called at least at {self.rate} Hz.
		This function controls the robot.
		'''
		self.forward()
		self.rotate()

		self.forward()
		self.rotate()

		self.forward()
		self.rotate()

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