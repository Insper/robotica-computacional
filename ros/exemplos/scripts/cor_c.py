#! /usr/bin/env python3
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
import cormodule

erro = None

def recebe_erro(msg):
    global erro
    erro = msg.data

	
if __name__=="__main__":
    rospy.init_node("controle_cor")

        
    recebedor = rospy.Subscriber("/erro", Float64, recebe_erro)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    try:
        
        while not rospy.is_shutdown():


            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            if erro is not None:
                
                if (erro > 0):
                    print("Direita")            
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                if (erro < 0):
                    print("Esquerda")
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
            
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
