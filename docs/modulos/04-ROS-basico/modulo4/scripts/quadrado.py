#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import ???
import numpy as np

""" 
Running
	roslaunch my_simulation novas_formas.launch
	rosrun modulo4 quadrado.py
"""

class Control():
	def __init__(self):
		super().__init__()

		# Subscribers
		
		# Publishers
		self.cmd_vel_pub = rospy.Publisher(???, ???, queue_size=3)

		self.cmd_vel_pub.publish(???())
	
	def forward(self, distance: float = 0.5, vel = ???()) -> None:
		???

	def rotate(self, angle: float = np.pi/2, vel = ???()) -> None:
		???

	def control(self):
		'''
		This function is called at least at {self.rate} Hz.
		This function controls the robot.
		'''
		???

		
def main():
	rospy.init_node('Controler')
	control = Control()
	rospy.sleep(2)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()