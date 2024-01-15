#!/usr/bin/env python3

import rospy
from std_msgs.msg import ???

""" 
Rode cada linha em um terminal diferente
	roscore
	rosrun modulo4 publisher.py
	rosrun modulo4 subscriber.py
"""

class Subscriber():
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz

		# Subscribers
		self.sub = rospy.Subscriber(???, ???, self.callback)

		# Publishers
	
	def callback(self, msg)	-> None:
		???

	def control(self) -> None:
		'''
		This function is called at least at {self.rate} Hz.
		'''
		???
		rospy.loginfo(???)
		
		self.rate.sleep()

def main():
	rospy.init_node('subscribers')
	control = Subscriber()
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()

