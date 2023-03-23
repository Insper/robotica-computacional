#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np

from geometry_msgs.msg import ???
from sensor_msgs.msg import LaserScan

""" 
Running
	roslaunch my_simulation paralelas.launch
	rosrun aps4 indeciso.py
"""

class Control():
	def __init__(self):

		self.rate = rospy.Rate(250) # 250 Hz
		# Subscribers
		self.laser_subscriber = rospy.Subscriber('/scan',LaserScan, self.laser_callback)
		
		# Publishers
		self.cmd_vel_pub = rospy.Publisher(???, ???, queue_size=3)

		self.cmd_vel_pub.publish(???)
		
	def laser_callback(self, msg: LaserScan) -> None:
		self.laser_msg = np.array(msg.ranges).round(decimals=2) # Converte para np.array e arredonda para 2 casas decimais
		self.laser_msg[self.laser_msg == 0] = np.inf

		print(f'Leitura diretamente na frente: {self.laser_msg[0]}')
		print("Faixa valida: ", msg.range_min , " - ", msg.range_max )
	
	def control(self) -> None:
		'''
		This function is called at least at {self.rate} Hz.
		This function controls the robot.
		'''
		???

		self.rate.sleep()


def main():
	rospy.init_node('indeciso')
	control = Control()
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()