#!/usr/bin/env python3

import rospy
from std_msgs.msg import ???

""" 
Running each line in a different terminal
	roslaunch roscore
	rosrun modulo4 publisher.py
	rosrun modulo4 subscriber.py
"""

class Publisher():
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz

		# Subscribers

		# Publishers
		self.pub = rospy.Publisher(??, ???, queue_size=10)
	
	def control(self) -> None:
		'''
		This function is called at least at {self.rate} Hz.
		'''
		
		tempo = ???
		msg = ???
		rospy.loginfo(???)
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

