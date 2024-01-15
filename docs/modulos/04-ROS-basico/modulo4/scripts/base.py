#!/usr/bin/env python3

import rospy

""" 
Rode cada linha em um terminal diferente
	rosrun modulo4 base.py
"""

class Control():
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz

		# Subscribers

		# Publishers
	
	def control(self) -> None:
		'''
		This function is called at least at {self.rate} Hz.
		'''
		
		self.rate.sleep() # Sleeps the remaining time to keep the rate

def main():
	rospy.init_node('Controler')
	control = Control()
	rospy.sleep(1) # Espera 1 segundo para que os publishers e subscribers sejam criados

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()

"""
roslaunch my_simulation caixas.launch
"""

