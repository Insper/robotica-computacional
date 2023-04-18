#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

""" 
Rode cada linha em um terminal diferente
	roscore
	rosrun modulo4 publisher.py
	rosrun modulo4 subscriber.py
"""

class Publisher():
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz
		self.contador = 0

		# Subscribers

		# Publishers
		self.pub = rospy.Publisher('publisher', String, queue_size=10)
	
	def control(self) -> None:
		'''
		This function is called at least at {self.rate} Hz.
		'''
		
		tempo = rospy.Time.now().to_sec()
		self.contador += 0
		msg = String()
		msg.data = f'{tempo} {self.count}'
		rospy.loginfo(f'Ola, são {tempo} e estou publicando pela {self.contador} vez')
		self.pub.publish(msg) # Publica a mensagem
		self.rate.sleep()

def main():
	rospy.init_node('publisher')
	control = Publisher()
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()

