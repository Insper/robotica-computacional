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
import os

import cv2.aruco as aruco
import sys

#--- Define Tag de teste
id_to_find  = 200
marker_size  = 20 #- [cm]

#--- Get the camera calibration path
calib_path  = os.path.abspath(os.getcwd())
camera_matrix   = np.loadtxt(calib_path+'/cameraMatrix_realsense.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'/cameraDistortion_realsense.txt', delimiter=',')

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 1000

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

#converte a msg do ROS para OpenCV
bridge = CvBridge() 
cv_image = None
scan_dist = 0

def scaneou(dado):
	scan_dist = dado.ranges[0]*100
	return scan_dist


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    # Recebe o frame comprimido via ROS
    cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8") 
    #Converte a imagem pra tons de cinza
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    #Cria uma lista de pontos e ids diponiveis
    corners, ids, pontosdescartaveis = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #exibe a lista de ids
    print(ids)

    #Se a lista de ids nao estiver vazia
    if ids is not None:

        #-- ret recebe os dados de posicao, rvec e tvec 
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        #-- rvec e uma lista contendo os vetores de rotacao  -----> rvec = [[rvec_1], [rvec_2], ...] 
        #-- tvec e uma lista contendo os vetores de translação ---------> tvec = [[tvec_1], [tvec_2], ...] vetor de translação
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        #-- Desenha um retanculo e exibe Id do marker encontrado
        aruco.drawDetectedMarkers(cv_image, corners, ids) 
        cv2.drawFrameAxes(cv_image, camera_matrix, camera_distortion, rvec, tvec ,0.03)



        #-- Exibe tvec --> vetor de tanslação em x y z
        str_position = "Marker x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
        print(str_position)
        #-- Desenha na imagem o vetor de tanslação em x y z
        cv2.putText(cv_image, str_position, (0, 100), font, 1, (0, 0, 0), 1, cv2.LINE_AA)

        ##############----- Referencia dos Eixos------###########################
        # Desenha a linha referencia em X
        cv2.line(cv_image, (cv_image.shape[1]//2,cv_image.shape[0]//2), ((cv_image.shape[1]//2 + 50),(cv_image.shape[0]//2)), (0,0,255), 5) 
        # Desenha a linha referencia em Y
        cv2.line(cv_image, (cv_image.shape[1]//2,cv_image.shape[0]//2), (cv_image.shape[1]//2,(cv_image.shape[0]//2 + 50)), (0,255,0), 5) 	
        
        #####################---- Distancia Euclidiana ----#####################
        # Calcula a distancia usando apenas a matriz tvec, matriz de tanslação
        distance = np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
        distancenp = np.linalg.norm(tvec)

        #-- Print distance
        str_dist = "Dist aruco=%4.0f  dis.np=%4.0f"%(distance, distancenp)
        print(str_dist)
        cv2.putText(cv_image, str_dist, (0, 15), font, 1, (0, 0, 0), 1, cv2.LINE_AA)


    # Exibe tela
    cv2.imshow("Camera", cv_image)
    cv2.waitKey(1)

	
if __name__=="__main__":

    #inicializa o node de conexao com o ROS
	rospy.init_node("aruco")

    #Chama a função roda_todo_frame sempre que chegar um frame novo
	recebe_imagem = rospy.Subscriber("/camera/image/compressed" , CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    #Loop do ROS
	while not rospy.is_shutdown():
		rospy.sleep(0.1)

