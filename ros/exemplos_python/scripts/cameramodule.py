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


class CameraNode(object):
    def __init__(self, use_webcam=False, check_delay=True, debug=False):
        self.bridge = CvBridge()
        self.cv_image = None
        self.debug_image = None
        self.debug = debug
        self.atraso = 1.5E9  # 1 segundo e meio. Em nanossegundos
        self.check_delay = check_delay  # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. Descarta imagens que chegam atrasadas demais

        # Para usar a Raspberry Pi
        topico_raspberry_camera = "/raspicam_node/image/compressed"
        # Para usar a webcam
        topico_webcam = "/cv_camera/image_raw/compressed"

        topico_imagem = topico_raspberry_camera
        if use_webcam:
            topico_imagem = topico_webcam

        self.recebedor = rospy.Subscriber(topico_imagem, CompressedImage, self.roda_todo_frame, queue_size=4, buff_size = 2**24)
        print("Usando ", topico_imagem)

    def roda_todo_frame(self, imagem):
        now = rospy.get_rostime()
        imgtime = imagem.header.stamp
        lag = now-imgtime
        delay = lag.nsecs
        print("Delay ", "{:.3f}".format(delay/1.0E9))
        if delay > self.atraso and self.check_delay==True:
            print("Descartando por causa do delay do frame:", delay)
            return
        try:
            antes = time.clock()
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            if self.debug:
                self.debug_image = self.cv_image.copy()

            self.processa_frame(self.cv_image)

            depois = time.clock()

            cv2.imshow("Camera", self.cv_image)
            if self.debug:
                cv2.imshow("Debug", self.debug_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)

    def processa_frame(self, frame):
        '''
        Recebe uma imagem do OpenCV e realiza o processamento necessário.
        Sobrescreva este método para fazer o que você quiser.
        '''
        pass
