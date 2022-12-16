#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

topico_odom = "/odom"


def recebeu_leitura(dado):
    """
        Grava nas variáveis x,y,z a posição extraída da odometria
    """
    # Salva os dados de posicao capturados pela odometria
    x = dado.pose.pose.position.x
    y = dado.pose.pose.position.y
    z = dado.pose.pose.position.z
    print("x {} y {} z {}".format(x, y, z))

if __name__=="__main__":

    rospy.init_node("print_odom")

    # Cria um subscriber que chama recebeu_leitura sempre que houver nova odometria
    recebe_scan = rospy.Subscriber(topico_odom, Odometry , recebeu_leitura)
    
    #loop do ROS
    while not rospy.is_shutdown():
        # sleep pra nao floodar o terminal
        rospy.sleep(1)
