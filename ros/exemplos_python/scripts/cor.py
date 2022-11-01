#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda", "Andrew Kurauchi"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule
import cameramodule


class CorNode(cameramodule.CameraNode):
    def __init__(self, use_webcam=False, check_delay=True, debug=False):
        super(CorNode, self).__init__(use_webcam, check_delay, debug)
        self.media = []
        self.centro = []
        self.area = 0.0 # Variavel com a area do maior contorno

    def processa_frame(self, frame):
        self.media, self.centro, self.area = cormodule.identifica_cor(frame, self.debug)

    def media_encontrada(self):
        return self.media is not None and len(self.media) != 0 and self.centro is not None and len(self.centro) != 0

    def dif_x(self):
        if self.media_encontrada():
            return self.media[0] - self.centro[0]
        return None

    def dif_y(self):
        if self.media_encontrada():
            return self.media[1] - self.centro[1]
        return None


if __name__=="__main__":
    # Inicializa nó do ROS
    rospy.init_node("cor")

    cor_node = CorNode()
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    try:
        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            if cor_node.media_encontrada():
                dif_x = cor_node.dif_x()
                dif_y = cor_node.dif_y()
                if math.fabs(dif_x) < 30:  # Se a media estiver muito proxima do centro anda para frente
                    vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
                else:
                    if dif_x > 0: # Vira a direita
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
                    else: # Vira a esquerda
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
            velocidade_saida.publish(vel)
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


