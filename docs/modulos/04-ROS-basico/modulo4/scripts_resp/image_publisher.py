#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy

import numpy as np
import cv2
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge,CvBridgeError
import numpy as np

""" 
Rode cada linha em um terminal diferente
	roslaunch my_simulation pista_s2.launch
	rosrun modulo4 image_publisher.py
"""

class ImagePublisher():
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz

		# HSV Filter
		self.lower_hsv = np.array([100,60,60],dtype=np.uint8) # Blue
		self.upper_hsv = np.array([140,255,255],dtype=np.uint8)
		self.kernel = np.ones((5,5),np.uint8)

		# Image
		self.point = Point()
		self.point.x = -1
		self.point.y = -1

		# Subscribers
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/camera/image/compressed',CompressedImage,self.image_callback,queue_size=1,buff_size = 2**24)
		
		# Publishers
		self.image_pub = rospy.Publisher("/image_publisher/", Image, queue_size=1)
		self.point_pub = rospy.Publisher("/center_publisher/", Point, queue_size=1)
		
	def image_callback(self, msg: CompressedImage) -> None:
		"""
		Callback function for the image topic
		"""
		try:
			cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		self.color_segmentation(cv_image) # Processamento da imagem

		if self.point.x != -1:
			cv_image = cv2.circle(cv_image, (self.point.x,self.point.y), 5, (0,0,255), -1)

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


	def color_segmentation(self,bgr: np.ndarray) -> None:
		""" 
		Use HSV color space to segment the image and find the center of the object.

		Args:
			bgr (np.ndarray): image in BGR format
		"""
		
		hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

		mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN, self.kernel)
		mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE, self.kernel)

		# find contours
		contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		if len(contours) > 0:
			# find contour with max area
			cnt = max(contours, key = lambda x: cv2.contourArea(x))

			# Find the center
			M = cv2.moments(cnt)
			self.point.x = int(M['m10']/M['m00'])
			self.point.y = int(M['m01']/M['m00'])
		else:
			self.point.x = -1
			self.point.y = -1

	def control(self) -> None:
		'''
		This function is called at least at {self.rate} Hz.
		'''
		
		self.point_pub.publish(self.point)
		if self.point.x != -1:
			print(f'O centro do creeper esta em ({self.point.x},{self.point.y})')
		else:
			print('Nenhum creeper encontrado')
		
		self.rate.sleep()

def main():
	rospy.init_node('Controler')
	control = ImagePublisher()
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()