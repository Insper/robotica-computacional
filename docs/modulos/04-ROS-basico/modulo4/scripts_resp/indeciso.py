#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

""" 
Rode cada linha em um terminal diferente
	roslaunch my_simulation paralelas.launch
	rosrun modulo4 indeciso.py
"""

class Control():
	def __init__(self):

		self.rate = rospy.Rate(250) # 250 Hz
		
		# Subscribers
		self.laser_subscriber = rospy.Subscriber('/scan',LaserScan, self.laser_callback)
		
		# Publishers
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

		self.cmd_vel_pub.publish(Twist())
		
	def laser_callback(self, msg: LaserScan) -> None:
		self.laser_msg = np.array(msg.ranges).round(decimals=2) # Converte para np.array e arredonda para 2 casas decimais
		self.laser_msg[self.laser_msg == 0] = np.inf

		self.frente = list(self.laser_msg)[0:5] + list(self.laser_msg)[-5:]

		print(f'Leitura na frente: {self.laser_msg[0]}')
		print("Faixa valida: ", msg.range_min , " - ", msg.range_max )
	
	def control(self) -> None:
		'''
		This function is called at least at {self.rate} Hz.
		This function controls the robot.
		'''
		vel = Twist()
		valor_min = np.min(self.frente)

		if valor_min > 1.05:
			vel.linear.x = 0.5
		elif valor_min < 0.95:
			vel.linear.x = -0.5
		else:
			vel.linear.x = 0

		self.cmd_vel_pub.publish(vel)
		self.rate.sleep()


def main():
	rospy.init_node('indeciso')
	control = Control()
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()