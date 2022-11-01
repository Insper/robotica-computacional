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
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import Float64

dados_laser = None
min_laser = 0
max_laser = 0

def recebe_scan(msg):
    global dados_laser
    global min_laser
    global max_laser
    dados_laser = msg.ranges
    min_laser = msg.range_min
    max_laser = msg.range_max


class MaquinaEstado:

    def __init__(self):

        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.estado = "PROCURA AZUL"

        self.erro_centro = rospy.Subscriber("/erro_centro", Float64, self.erro_callback )

        # Variavel do sensor de erro
        self.erro = 0

    def erro_callback(self, msg):
        self.erro = msg.data

    def controle_procura_azul(self):
        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

        if (self.erro > 0):
            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
        if (self.erro < 0):
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
        
        self.velocidade_saida.publish(vel)

        if  - 10 < self.erro <  + 10:
            return "AVANCA AZUL"
            
        return "PROCURA AZUL"

    def controle_avanca_azul(self):
        
        vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))

        if  self.erro < - 10 or self.erro > + 10:
            vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
            self.velocidade_saida.publish(vel)
            return "PROCURA AZUL"

        if  dados_laser[0] < .3:
            vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))

        self.velocidade_saida.publish(vel)

        return "AVANCA AZUL"


    def processa_dados(self):

        if self.estado == "PROCURA AZUL":
            self.estado = self.controle_procura_azul()
        elif self.estado == "AVANCA AZUL":
            self.estado = self.controle_avanca_azul()


if __name__=="__main__":
    rospy.init_node("cor_controle")

    recebedor_laser = rospy.Subscriber("/scan", LaserScan, recebe_scan)

    maquina_estados = MaquinaEstado()

    try:

        while not rospy.is_shutdown():
            maquina_estados.processa_dados()
            
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
