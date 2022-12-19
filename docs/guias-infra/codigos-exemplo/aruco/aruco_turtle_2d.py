#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import cv2.aruco as aruco
import numpy as np 
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]

bridge = CvBridge()
cv_image = None

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    ''' Função chamada sempre que chega um novo frame, e retorna a imagem do OpenCV '''
    global cv_image
    
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
    except CvBridgeError as e:	print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("processa_imagem")

    # topico_imagem = "/kamera"
    topico_imagem = "/camera/image/compressed" # Use para robo virtual
    
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    while not rospy.is_shutdown():
        
        if cv_image is not None:
            frame = cv_image
            # Our operations on the frame come here
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #--- Define the aruco dictionary
            aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
            
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict) #, parameters=parameters)

            if ids is not None:
                for i in range(len(ids)):
                    print('ID: {}'.format(ids[i]))
                    
                    for c in corners[i]: 
                        for canto in c:
                            print("Corner {}".format(canto))
                    
                aruco.drawDetectedMarkers(frame, corners, ids)

            cv2.imshow('frame',frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break