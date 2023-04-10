#!/usr/bin/env python3

import rospy
from std_msgs.msg import ???

""" 
Running each line in a different terminal
	roslaunch roscore
	rosrun aps4 publisher.py
	rosrun aps4 subscriber.py
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

