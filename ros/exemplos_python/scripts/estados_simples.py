#! /usr/bin/env python
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


LIMITE = 200
ANG_SPEED = 0.2

## Classes - estados
"""
Classe simples - estado que gira até o limite depois termina
"""
class Girando(smach.State):
    def __init__(self, velocidade_saida):
        smach.State.__init__(self, outcomes=['fim', 'girando'])
        self.numero_voltas = 0
        self.velocidade_saida = velocidade_saida

    def execute(self, userdata):
        self.numero_voltas +=1
        if self.numero_voltas < LIMITE:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ANG_SPEED))
            self.velocidade_saida.publish(vel)
            return 'girando'
        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            self.velocidade_saida.publish(vel)
            return 'fim'


# main
def main():
    rospy.init_node('unico_estado')

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    # Cria uma máquina de estados
    sm = smach.StateMachine(outcomes=['fim_geral'])

    # Preenche a Smach com os estados
    with sm:
        smach.StateMachine.add('GIRANDO', Girando(velocidade_saida),
            transitions={'girando': 'GIRANDO',
            'fim':'fim_geral'})

    # Executa a máquina de estados
    outcome = sm.execute()

    print("Execute finished")


if __name__ == '__main__':
    print("Main")
    main()
