#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


distancia = 20.0

def scaneou(dado):
	global distancia # declaro que uso a distancia externa
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	ranges = np.array(dado.ranges).round(decimals=2)
	distancia = ranges[0]

	
	


if __name__=="__main__":

	rospy.init_node("exemplos_5af")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	reverse = Twist(Vector3(-0.3, 0, 0), Vector3(0, 0, 0))
	forward = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))

	vel = forward

	while not rospy.is_shutdown():
		print("Oeee")
		
		velocidade_saida.publish(vel)

		if distancia < 2.5:
			# pode ser uma boa frear antes
			vel = reverse	
		rospy.sleep(0.1)


