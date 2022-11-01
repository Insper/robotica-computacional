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
import smach
import smach_ros

from cor import CorNode


TOLERANCIA_X = 50
ANG_SPEED = 0.4


## Classes - estados
class Girando(smach.State):
    def __init__(self, cor_node, velocidade_saida):
        smach.State.__init__(self, outcomes=['alinhou', 'girando'])
        self.cor_node = cor_node
        self.velocidade_saida = velocidade_saida

    def execute(self, userdata):
        dif_x = self.cor_node.dif_x()
        if dif_x is None:
            return 'girando'

        if  dif_x > TOLERANCIA_X:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ANG_SPEED))
            self.velocidade_saida.publish(vel)
            return 'girando'
        if dif_x < -TOLERANCIA_X:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ANG_SPEED))
            self.velocidade_saida.publish(vel)
            return 'girando'
        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            self.velocidade_saida.publish(vel)
            return 'alinhou'


class Centralizado(smach.State):
    def __init__(self, cor_node, velocidade_saida):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado'])
        self.cor_node = cor_node
        self.velocidade_saida = velocidade_saida

    def execute(self, userdata):
        dif_x = self.cor_node.dif_x()
        if dif_x is None:
            return 'alinhado'
        if  dif_x > TOLERANCIA_X:
            return 'alinhando'
        if dif_x < -TOLERANCIA_X:
            return 'alinhando'
        else:
            vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            self.velocidade_saida.publish(vel)
            return 'alinhado'


# main
def main():
    cor_node = CorNode()
    rospy.init_node('cor_estados')

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminei'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GIRANDO', Girando(cor_node, velocidade_saida),
                                transitions={'girando': 'GIRANDO',
                                'alinhou':'CENTRO'})
        smach.StateMachine.add('CENTRO', Centralizado(cor_node, velocidade_saida),
                                transitions={'alinhando': 'GIRANDO',
                                'alinhado':'CENTRO'})


    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    main()
