#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError


import cv2.aruco as aruco
import sys

#--- Define Tag de teste
id_to_find  = 200
marker_size  = 25 #- [cm]
#id_to_find  = 22
#marker_size  = 3 #- [cm]
# 


#--- Get the camera calibration path
calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos211/scripts/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
#parameters.adaptiveThreshWinSizeMax = 1000

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

bridge = CvBridge() #converte a msg do ROS para OpenCV
cv_image = None
scan_dist = 0

def scaneou(dado):
	#print("scan")
	global scan_dist 
	scan_dist = dado.ranges[0]*100
	return scan_dist


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	#print("frame")
	
	try:
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8") # imagem compressed
		#cv_image = bridge.imgmsg_to_cv2(imagem, "bgr8") 			# imagem não compressed
		#cv_image = cv2.resize(cv_image,(cv_image.shape[1]*2,cv_image.shape[0]*2)) # resize image se necessario
		
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		
		if ids is not None and ids[0] == id_to_find :
			#-- ret = [rvec, tvec, ?]
			#-- array of rotation and position of each marker in camera frame
			#-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
			#-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
			ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

			#-- Unpack the output, get only the first
			rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

			#-- Desenha um retanculo e exibe Id do marker encontrado
			aruco.drawDetectedMarkers(cv_image, corners, ids) 
			aruco.drawAxis(cv_image, camera_matrix, camera_distortion, rvec, tvec, 1)
			
			# Calculo usando distancia Euclidiana 
			distance = np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)

			#-- Print the tag position in camera frame
			str_position = "Marker x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
			print(str_position)
			cv2.putText(cv_image, str_position, (0, 100), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

			#-- Print the tag position in camera frame
			str_dist = "Dist aruco=%4.0f  scan=%4.0f"%(distance, scan_dist)
			print(str_dist)
			cv2.putText(cv_image, str_dist, (0, 15), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

			# Linha referencia em X
			cv2.line(cv_image, (cv_image.shape[1]/2,cv_image.shape[0]/2), ((cv_image.shape[1]/2 + 50),(cv_image.shape[0]/2)), (0,0,255), 5) 
			# Linha referencia em Y
			cv2.line(cv_image, (cv_image.shape[1]/2,cv_image.shape[0]/2), (cv_image.shape[1]/2,(cv_image.shape[0]/2 + 50)), (0,255,0), 5) 	


			cv2.putText(cv_image, "%.1f cm -- %.0f deg" % ((tvec[2]), (rvec[2] / 3.1415 * 180)), (0, 230), font, 1, (244, 244, 244), 1, cv2.LINE_AA)

		# Exibe tela
		cv2.imshow("Camera", cv_image)
		cv2.waitKey(1)
	except CvBridgeError as e:
		print('ex', e)
	
if __name__=="__main__":
	rospy.init_node("aruco")

	topico_imagem = "/camera/image/compressed"
	recebe_imagem = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

	# Teste com imagem não compressed
	#topico_imagem = "/camera/image"
	#recebe_imagem = rospy.Subscriber(topico_imagem, Image, roda_todo_frame, queue_size=4, buff_size = 2**24)
	
	print("Usando ", topico_imagem)

	# validação da distância usando laser scan
	#recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	
	
	try:

		while not rospy.is_shutdown():
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
