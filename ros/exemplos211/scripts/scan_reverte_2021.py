#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

colidiu = False

def scaneou(dado):
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	global colidiu
	leituras = np.array(dado.ranges).round(decimals=2)
	
	if leituras[0] < 0.7:
		colidiu = True

	
if __name__=="__main__":

	rospy.init_node("scan_reverte_aula")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	v = 0.2

	zero = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))


	while not rospy.is_shutdown():
		print("Oeee")

		if not colidiu: 
			velocidade = Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(0.01)
		elif colidiu: 
			velocidade_saida.publish(zero)
			rospy.sleep(2)		
			velocidade = Twist(Vector3(-v, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(2)









