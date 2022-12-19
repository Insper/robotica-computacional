#! /usr/bin/env python3
# -*- coding:utf-8 -*-

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
import smach
import smach_ros
import cormodule




class Cor():
	#incializando as variaveis
	def __init__(self):
		#inicializando o node com o ROS
		rospy.init_node('cor')
		
		self.cv_image = None
		#O CvBridge converte mensagens de imagem do ROS em imagem do Opencv.
		self.bridge = CvBridge()
		#Chama a função roda_todo_frame sempre que chegar um frame novo
		recebedor = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.roda_todo_frame, queue_size=10, buff_size = 2**24)
		#Cria o publisher para publicar os dados de velocidade no robo 
		self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
		
		#listas com os pontos de centro da imagem, ponto medio da mascara, area total da mascara e tolerancia no eixo X
		self.centro = []
		self.media = []
		self.area = 0.0
		self.tolerancia_x = 30
		self.area_objeto = 3000
	# funcao que roda toda vez que chega um frame novo
	def roda_todo_frame(self,imagem):

		try:
			#faz a conversao da mensagem ROS em imagem opencv de 8 bits de tamanho do tipo BGR 
			self.cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
			#recebe os parametros de media, centro e area do cormodule.py, onde estao definidos os valores da mascara
			self.media, self.centro, self.area = cormodule.identifica_cor(self.cv_image)
			#exibe os valores de centro e media da imagem
			print("media {},{}".format(*self.media))
			print("centro {},{}".format(*self.centro))
			#faz o display da imagem
			cv2.imshow("Camera", self.cv_image)
		except CvBridgeError as e:
			#se houver algum erro na conversao da mensagem ROS para o opencv, exibe o erro no print
			print(e)

	#funcao que controla as acoes do robo
	def control(self):
		#chama a funcao girando, que faz o robo girar em torno de si mesmo ate achar um objeto laranja
		encontrou = self.girando()

		#se o robo encontrar alguma coisa laranja, entra no if
		if encontrou == True:
			#chama a funcao centralizou que faz o ajuste fino do robo, na tentativa de centralizar no objeto laranja encontrado
			centralizou = self.centralizando()
			if centralizou == True:
				print("Centralizado e alinhado na caixa laranja <3 ")
			else:
				print("Precisamos alinhar com o centro da caixa laranja, ajustando o angulo do robo ")
		else:
			print("Precisamos encontrar uma caixa laranja, girando")

	def girando(self):

		if self.media is None or len(self.media)==0:
			return False
		if  (math.fabs(self.media[0]) >= self.area_objeto) and (math.fabs(self.media[0]) > math.fabs(self.centro[0] + self.tolerancia_x)):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.5))
			self.velocidade_saida.publish(vel)
			return False
		if (math.fabs(self.media[0]) >= self.area_objeto) and (math.fabs(self.media[0]) - self.tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
			self.velocidade_saida.publish(vel)
			return False
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			self.velocidade_saida.publish(vel)
			return True


	def centralizando(self):
		if self.media is None:
			return False
		if  (math.fabs(self.media[0]) >= self.area_objeto) and (math.fabs(self.media[0]) + self.tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.25))
			self.velocidade_saida.publish(vel)
			return False
		if (math.fabs(self.media[0]) >= self.area_objeto) and (math.fabs(self.media[0]) - self.tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.25))
			self.velocidade_saida.publish(vel)
			return False
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			self.velocidade_saida.publish(vel)
			return True

# Main loop
if __name__=="__main__":
    cor = Cor()

	#loop do ROS
    while not rospy.is_shutdown():
        cor.control()