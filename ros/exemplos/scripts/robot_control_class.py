#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import time

"""
Topicos: velocidade
type: Twist
braço
/cmd_vel

Topicos: braço e garra
type: std_msgs/Float64
braço
/joint1_position_controller/command
garra
/joint2_position_controller/command

Topicos: sersor lidar
type: LaserScan
laser_callback
/scan
"""

class RobotControl(object):

	def __init__(self):
		rospy.init_node('mybot_turtle_insper', anonymous=True)
		rospy.loginfo("Turtlebot INSPER...")

		self.vel_publisher = rospy.Publisher(
			'/cmd_vel',
			Twist, 
			queue_size=3)

		self.braco_publisher = rospy.Publisher(
            '/joint1_position_controller/command',
            Float64,
            queue_size=1)
		self.garra_publisher = rospy.Publisher(
            '/joint2_position_controller/command',
            Float64,
            queue_size=1)
		
		self.laser_subscriber = rospy.Subscriber(
			'/scan', 
			LaserScan, 
			self.laser_callback)

		self.cmd = Twist()
		self.laser_msg = LaserScan()
		
		self.ctrl_c = False
		self.rate = rospy.Rate(1) # 1Hz
		rospy.on_shutdown(self.shutdownhook)

	def move_joints(self, braco, garra):
		
		pos_braco = Float64()
		pos_braco.data = braco
		pos_garra = Float64()
		pos_garra.data = garra
		
		self.braco_publisher.publish(pos_braco)
		self.garra_publisher.publish(pos_garra)
		self.rate.sleep()

	def move_joints_init(self):
		pos_braco = Float64()
		pos_braco.data = -1.5
		pos_garra = Float64()
		pos_garra.data = 0
		
		self.braco_publisher.publish(pos_braco)
		self.garra_publisher.publish(pos_garra)
		self.rate.sleep()
	
	def move_joints_sobe(self):
		pos_braco = Float64()
		pos_braco.data = 1.5
		pos_garra = Float64()
		pos_garra.data = 0
		
		self.braco_publisher.publish(pos_braco)
		self.garra_publisher.publish(pos_garra)
		self.rate.sleep()
  	
	def publish_once_in_cmd_vel(self):
		"""
		This is because publishing in topics sometimes fails the first time you publish.
		In continuos publishing systems there is no big deal but in systems that publish only
		once it IS very important.
		"""
		while not self.ctrl_c:
			connections = self.vel_publisher.get_num_connections()
			if connections > 0:
				self.vel_publisher.publish(self.cmd)
				#rospy.loginfo("Cmd Published")
				break
			else:
				self.rate.sleep()

	def shutdownhook(self):
		# works better than the rospy.is_shutdown()
		self.stop_robot()
		self.ctrl_c = True

	def laser_callback(self, msg):
		self.laser_msg = msg

	def get_laser(self, pos):
		#time.sleep(1)
		return self.laser_msg.ranges[pos]

	def stop_robot(self):
		self.cmd.linear.x = 0.0
		self.cmd.angular.z = 0.0
		self.publish_once_in_cmd_vel()

	def move_straight(self):

		# Initilize velocities
		self.cmd.linear.x = 0.1
		self.cmd.linear.y = 0
		self.cmd.linear.z = 0
		self.cmd.angular.x = 0
		self.cmd.angular.y = 0
		self.cmd.angular.z = 0

		# Publish the velocity
		self.publish_once_in_cmd_vel()

	def turn(self, clockwise, speed, time):

		# Initilize velocities
		self.cmd.linear.x = 0
		self.cmd.linear.y = 0
		self.cmd.linear.z = 0
		self.cmd.angular.x = 0
		self.cmd.angular.y = 0


		if clockwise == "clockwise":
			self.cmd.angular.z = -speed
		else:
			self.cmd.angular.z = speed

		i = 0
		# loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
		while (i <= time):

			# Publish the velocity
			self.vel_publisher.publish(self.cmd)
			i += 1
			self.rate.sleep()

		# set velocity to zero to stop the robot
		self.stop_robot()

		s = "Rotacionando o robô no sentido " + clockwise + " por " + str(time) + " segundos"
		return s

#
#if __name__ == '__main__':
#	#rospy.init_node('robot_control_node', anonymous=True)
#	robotcontrol_object = RobotControl()
#	try:
#		robotcontrol_object.move_straight()
#
#	except rospy.ROSInterruptException:
#		pass
#