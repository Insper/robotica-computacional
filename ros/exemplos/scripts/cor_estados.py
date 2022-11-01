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
from cv_bridge import CvBridge, CvBridgeError
import cormodule


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = rospy.Time.now()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real
        media, centro, maior_area =  cormodule.identifica_cor(cv_image)
        depois = rospy.Time.now()
        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)


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

    def controle_procura_azul(self):
        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

        if len(media) != 0 and len(centro) != 0:
            print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
            print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))

            if (media[0] > centro[0]):
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
            if (media[0] < centro[0]):
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
            
            self.velocidade_saida.publish(vel)

            if  centro[0] - 10 < media[0] < centro[0] + 10:
                return "AVANCA AZUL"
            
        return "PROCURA AZUL"

    def controle_avanca_azul(self):
        
        vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))

        if  media[0] < centro[0] - 10 or media[0] > centro[0] + 10:
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
    rospy.init_node("cor")

    # topico_imagem = "/kamera"
    topico_imagem = "camera/image/compressed" # Use para robo virtual
    # topico_imagem = "/raspicam/image_raw/compressed" # Use para robo real
    
    # Para renomear a *webcam*
    #   Primeiro instale o suporte https://github.com/Insper/robot19/blob/master/guides/debugar_sem_robo_opencv_melodic.md
    #
    #	Depois faça:
    #	
    #	rosrun cv_camera cv_camera_node
    #
    # 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
    #
    # 
    # Para renomear a câmera simulada do Gazebo
    # 
    # 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
    # 
    # Para renomear a câmera da Raspberry
    # 
    # 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
    # 

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    print("Usando ", topico_imagem)

    recebedor_laser = rospy.Subscriber("/scan", LaserScan, recebe_scan)

    maquina_estados = MaquinaEstado()

    try:

        while not rospy.is_shutdown():
            maquina_estados.processa_dados()
            
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
