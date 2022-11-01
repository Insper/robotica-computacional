#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


import visao_module


bridge = CvBridge()

cv_image = None
media = []
centro = []

ids_possiveis_tags = [11,12,13,21,22,23] # Baseado no enunciado do projeto

area = 0.0 # Variavel com a area do maior contorno colorido

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos pela MobileNet

x = 0
y = 0
z = 0 

tfl = 0

tf_buffer = tf2_ros.Buffer()


def faz_transformacao(ref1, ref2):
    """Realiza a transformacao do ponto entre o referencial 1 e o referencial 2 
        retorna a trasnformacao

        Para saber todos os referenciais disponíveis veja o frames.pdf gerado por: 

        rosrun tf view_frames 
    """
    print(tf_buffer.can_transform(ref1, ref2, rospy.Time(0)))
    transf = tf_buffer.lookup_transform(ref1, ref2, rospy.Time(0))
    return transf

def decompoe(transf):
    """Recebe uma transformacao de sistemas de coordenadas e a converte em x,y,z e ângulo em RAD em relação a z"""
    # Separa as translacoes das rotacoes
    x = transf.transform.translation.x
    y = transf.transform.translation.y
    z = transf.transform.translation.z
    # ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
    # Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
    # no eixo X do robo (que e'  a direcao para a frente)
    t = transformations.translation_matrix([x, y, z])
    # Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
    r = transformations.quaternion_matrix([transf.transform.rotation.x, transf.transform.rotation.y, transf.transform.rotation.z, transf.transform.rotation.w])
    m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
    z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
    v2 = numpy.dot(m, z_marker)
    v2_n = v2[0:-1] # Descartamos a ultima posicao
    n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
    x_robo = [1,0,0]
    cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
    angulo_marcador_robo = math.acos(cosa)
    return x,y,z, angulo_marcador_robo

def insere_coords_dict(dici, x,y,z,alpha):
    " Insere coordenadas (vindas de um trasnformation) num dicionário para facilitar uso posterior"
    dici["x"] = x 
    dici["y"] = y 
    dici["z"] = z 
    dici["alpha"] = alpha
    dici["graus"] = math.degrees(alpha)

 
def recebe(msg):
    "Recebe o evento das tags alvar"
    global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
    global y
    global z
    global id

    frame_names = {"camera_rgb_optical_frame":"Coordenadas da câmera", "end_effector_link": "Cubo vermelho da mão", "base_link": "Base do robô"}
    frame_coords = {"camera_rgb_optical_frame":dict(), "end_effector_link": dict(), "base_link":dict()}


    for marker in msg.markers:
        id = marker.id
        marcador = "ar_marker_" + str(id)

        referenciais = frame_names.keys()
        for ref in referenciais: 
            transf = faz_transformacao(ref, marcador)
            if transf is not None:           
                xt, yt, zt, alpha_t = decompoe(transf)
                insere_coords_dict(frame_coords[ref], xt, yt, zt, alpha_t)

        for ref in frame_names.keys():
            print("\r")
            if ref in frame_coords.keys():
                print("Marcador ", id)
                print("No referencial :",ref, " que é ", end=None)
                print(frame_names[ref])
                for k in frame_coords[ref].keys():
                    print("%s %5.3f"%(k, frame_coords[ref][k]), end=" ")
                print()

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    # print("frame")
    global cv_image
    global media
    global centro
    global resultados

    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass
        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":

    print("""

ALERTA:







Para funcionar este programa *precisa* do Rviz rodando antes:

roslaunch turtlebot3_manipulation_moveit_config move_group.launch

roslaunch my_simulation rviz.launch

"""
    
    
    )

    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos


    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    # Exemplo de categoria de resultados MobileNEt
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        # Inicializando - por default gira no sentido anti-horário
        # vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        
        while not rospy.is_shutdown():
            for r in resultados:

                pass # Para evitar prints
            #   print(r)
            #velocidade_saida.publish(vel)

            if cv_image is not None:
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                cv2.imshow("cv_image no loop principal", cv_image)
                cv2.waitKey(1)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


