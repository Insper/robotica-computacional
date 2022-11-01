#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import time
from robot_control_class import RobotControl #importa classe

class Robo(RobotControl, object):
	def __init__(self):
		# pega os atributos da classe RobotControl
		super(Robo,self).__init__()

		print("inicializando...")
		self.pos_braco = 0
		self.pos_garra = -1

	def robotinicio(self):
		robo.stop_robot()
		robo.move_joints_init()
		print("...")
		print("estou pronto! ")

	def robotgarra(self):
		print("Exemplo move garra ")
		print("inicio")
		robo.move_joints_init()
		print("posição")
		robo.move_joints(self.pos_braco,self.pos_garra)
		print("sobe")
		robo.move_joints_sobe()



if __name__ == '__main__':
	robo = Robo() # cria o objeto
	robo.robotinicio() #inicializa o robo
	try:		
		while not robo.ctrl_c:

			robo.robotgarra()
			

	except rospy.ROSInterruptException:
		pass
	




