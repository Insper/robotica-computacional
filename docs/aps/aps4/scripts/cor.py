#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy

import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge,CvBridgeError
import numpy as np

""" 
Rode cada linha em um terminal diferente
	roslaunch my_simulation pista_s2.launch
	rosrun aps4 cor.py
"""

class Control():
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz
		
		self.robot_state = "procura"
		self.robot_machine = {
			"procura": self.procura,
			"aproxima": self.aproxima,
			"para": self.para
		}

		# Subscribers

		# Publishers
		
		# ...
		
	def laser_callback(self, msg: LaserScan) -> None:
		# ...

	
	def image_callback(self, msg: CompressedImage) -> None:
		# ...

	def color_segmentation(self,bgr: np.ndarray) -> None:
		""" 
		Use HSV color space to segment the image and find the center of the object.

		Args:
			bgr (np.ndarray): image in BGR format
		"""
		# ...

	def procura(self) -> None:
		"""
		Find the creeper
		"""
		# ...
	
	def aproxima(self) -> None:
		"""
		Go to the creeper
		"""
		# ...
	
	def para(self) -> None:
		"""
		Stop the robot
		"""
		# ...

	def control(self) -> None:
		'''
		This function is called at least at {self.rate} Hz.
		This function controls the robot.
		Não modifique esta função.
		'''
		self.twist = Twist()
		print(f'self.robot_state: {self.robot_state}')
		self.robot_machine[self.robot_state]()

		self.cmd_vel_pub.publish(self.twist)
		
		self.rate.sleep()

def main():
	rospy.init_node('Controler')
	control = Control()
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()

