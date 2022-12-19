# Encontrando a caixa laranja

Encontrar objetos usando técnicas de visão computacional é muito útil para que o robô consiga interagir com objetos do cenário, vamos utilizar mascaras e segmentação de objetos para encontrar a caixa laranja do cenário.

Na atividade 2 de robótica, apresentamos para você técnicas de detecção de cores e operações morfológicas para fazer a segmentação de imagens usando o opencv recomendo que volte lá para relembrar esses conceitos. 

Disponibilizamos o código  [cormodule.py](https://www.notion.so/cormodule-py-098a9f532b244555842ddf5e006404d2). Que é um código auxiliar usado para fazer a segmentação da máscara e contorno da maior área encontrada, o código retorna **media, centro,**  e **maior_contorno_area** da cor definida na máscara. Sugiro que leia o código atentamente para entender como o processo funciona. Esse código é usado como um módulo auxiliar, não executamos ele sozinho.

```python
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

#funcao que contorna a mascara encontrada
def cross(img_rgb, point, color, width,length):
    cv2.line(img_rgb, (point[0] - length//2, point[1]),  (point[0] + length//2, point[1]), color ,width, length)
    cv2.line(img_rgb, (point[0], point[1] - length//2), (point[0], point[1] + length//2),color ,width, length) 

#funcao que faz a mascara
def identifica_cor(frame):
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''

    # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao 
    # vermelho puro (H=0) estão entre H=-8 e H=8. 
    # Precisamos dividir o inRange em duas partes para fazer a detecção 
    # do vermelho:

    #transforma a imgem em hsv
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #define a mascara para as duas faixas de vermelho
    cor_menor = np.array([0, 80, 110])
    cor_maior = np.array([8, 255, 245])

    #segmenta a cor usanto o range definido para uma faixa
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    #segmenta a cor usando o range definido para a outra faixa
    cor_menor = np.array([150, 80, 110])
    cor_maior = np.array([180, 255, 245])

    #soma os dois ranges e salva na variavel segmentado_cor
    segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)

    # Note que a notacão do numpy encara as imagens como matriz, portanto o enderecamento é
    # linha, coluna ou (y,x)
    # Por isso na hora de montar a tupla com o centro precisamos considerar isso

    #calcula o centro da mascara
    centro = (frame.shape[1]//2, frame.shape[0]//2)

    maior_contorno = None
    maior_contorno_area = 0

    # A operação MORPH_CLOSE fecha todos os buracos na máscara menores 
    # que um quadrado 7x7. É muito útil para juntar vários 
    # pequenos contornos muito próximos em um só.
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    # Encontramos os contornos na máscara e selecionamos o de maior área
    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    # Encontrando o maior contorno
    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    # Encontramos o centro do contorno fazendo a média de todos seus pontos.
    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        cross(frame, centro, [255,0,0], 1, 17)
    else:
        media = (0, 0)

    # Representa a area e o centro do maior contorno no frame
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    cv2.putText(frame,"{:d} {:d}".format(*media),(20,100), 1, 4,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,"{:0.1f}".format(maior_contorno_area),(20,50), 1, 4,(255,255,255),2,cv2.LINE_AA)

    #faz o display da imagem
    cv2.imshow('seg', segmentado_cor)
    cv2.waitKey(1)

    return media, centro, maior_contorno_area
```

Com o código [cor.py](https://www.notion.so/cor-py-f0c360f93506407dbb30a79c691152a9), o robô gira em torno de si mesmo enquanto procura um objeto laranja, quando ele encontra esse objeto laranja, o robô faz o ajuste fino para centralizar em relação a esse objeto e para, analise o código de exemplo disponibilizado e:

- Ajuste os parâmetros necessários para que o robô procure apenas a caixa laranja, e ignore os outros objetos do cenário.
- Faça com que o robô se aproxime da caixa laranja e pare a uma distância de 1.5m da caixa (use o laser_scan para descobrir a distância que o robô está da caixa, se quiser.)

 

![Untitled](Encontrando%20a%20caixa%20laranja%20293cbea037e7440ebcb65c8342357726/Untitled.png)

```python
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
		self.tolerancia_x = 900

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
		if  math.fabs(self.media[0]) > math.fabs(self.centro[0] + self.tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.3))
			self.velocidade_saida.publish(vel)
			return False
		if math.fabs(self.media[0]) < math.fabs(self.centro[0] - self.tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.3))
			self.velocidade_saida.publish(vel)
			return False
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			self.velocidade_saida.publish(vel)
			return True

	def centralizando(self):
		if self.media is None:
			return False
		if  math.fabs(self.media[0]) > math.fabs(self.centro[0] + self.tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.1))
			self.velocidade_saida.publish(vel)
			return False
		if math.fabs(self.media[0]) < math.fabs(self.centro[0] - self.tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.1))
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
```