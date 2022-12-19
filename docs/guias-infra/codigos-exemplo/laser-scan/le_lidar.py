#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

# Funçao que le os dados do LIDAR
def scaneou(dado):
	#Define uma faixa valida de dados do sensor para trabalhar de 0.11 ate 3.5
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )

	#Cria uma lista com os dados capturados do sensor
	lista_dados = np.array(dado.ranges).round(decimals=2)
	
	#Percorre a lista de dados
	for leitura in lista_dados:
		#Se a leitura for um dado valido, exibe ele
		if leitura >= dado.range_min and leitura <= dado.range_max:
			print(leitura)



if __name__=="__main__":
	# inicializa o node com o ROS
	rospy.init_node("le_lidar")
	#Chama a função scaneou sempre que chegar um dado via ROS
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	# loop do ROS
	while not rospy.is_shutdown():
		# aguarda um tempinho, so pra nao floodar
		rospy.sleep(1)
