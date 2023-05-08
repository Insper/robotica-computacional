#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point
import numpy as np

""" 
Running
	roslaunch my_simulation novas_formas.launch
	rosrun aps4 goto.py

"""

class Control():
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz
		self.point = Point( x = 2, y = 1, z = 0)
		self.kp = 0.01

		self.robot_state = 'center'
		self.state_machine = {
			'center': self.center,
			'goto': self.goto,
			'stop': self.stop
		}

		# Subscribers
		self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odom_callback)

		# Publishers
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
	
	def odom_callback(self, data: Odometry):
		self.odom = data
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.z = data.pose.pose.position.z
		
		orientation_list = [data.pose.pose.orientation.x,
							data.pose.pose.orientation.y,
							data.pose.pose.orientation.z,
							data.pose.pose.orientation.w]

		self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

		# convert yaw from [-pi, pi] to [0, 2pi]
		self.yaw = self.yaw % (2*np.pi)

	def get_angular_error(self):
		x = self.point.x - self.x
		y = self.point.y - self.y
		theta = np.arctan2(y , x)

		self.distance = np.sqrt(x**2 + y**2)
		self.err = np.rad2deg(theta - self.yaw)
		self.err = (self.err + 180) % 360 - 180

		self.twist.angular.z = self.err * self.kp

	def center(self):
		self.get_angular_error()

		if abs(self.err) < 5:
			rospy.loginfo('Waypoint Centered')
			self.robot_state = 'goto'

	def goto(self):
		self.get_angular_error()

		if self.distance > 0.1:
			self.twist.linear.x = np.min([self.distance, 0.1])
		else:
			rospy.loginfo('Waypoint reached')
			self.robot_state = 'stop'
	
	def stop(self):
		self.twist = Twist()

	def control(self) -> None:
		'''
		This function is called at least at {self.rate} Hz.
		'''
		self.twist = Twist()
		print(f'{self.robot_state}')
		self.state_machine[self.robot_state]()

		self.cmd_vel_pub.publish(self.twist)
		self.rate.sleep() # Sleeps the remaining time to keep the rate

def main():
	rospy.init_node('GoTo')
	control = Control()
	rospy.sleep(1) # Espera 1 segundo para que os publishers e subscribers sejam criados

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()

