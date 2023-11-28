#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

""" 
	roslaunch my_simulation reuniao.launch
	rosrun pf-robcomp q3.py
"""

class Questao3():
	def __init__(self):

		self.rate = rospy.Rate(250) # 250 Hz
		
		# Subscribers

        # Publishers
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)


		# Maquina de estados
		self.robot_state = ""
		self.robot_machine = {
			"": self.nada
		}
	
	def nada(self):
		pass
	
	def control(self) -> None:
		'''
		This function is called at least at {self.rate} Hz.
		This function controls the robot.
		'''
		
		self.twist = Twist()
		print(f'self.robot_state: {self.robot_state}')
		self.robot_machine[self.robot_state]()

		self.cmd_vel_pub.publish(self.twist)

		self.rate.sleep()


def main():
	rospy.init_node('q3')
	control = Questao3()
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()