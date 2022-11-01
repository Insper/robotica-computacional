#! /usr/bin/env python3
# -*- coding:utf-8 -*-

"""
	Vai andando em frente ate encontrar algo a 1m detectado pelo laser e em seguida da re' 

	Feito na aula de 21/09/2020

	
"""

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

nao_bateu = True

def scaneou(dado):
	global nao_bateu
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	# print(np.array(dado.ranges).round(decimals=2))
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))
	print("leitura em zero grau", dado.ranges[0])

	if dado.ranges[0]<= 1.0:
		nao_bateu = False

if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	vel = 0.4


	while not rospy.is_shutdown():
		print("Oeee")
		## ir em frente enquanto nao houver objeto a 1m ou menos
		if nao_bateu:
			velocidade = Twist(Vector3(vel, 0, 0), Vector3(0, 0, 0))
		else:
			velocidade = Twist(Vector3(-vel, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		rospy.sleep(0.01)


