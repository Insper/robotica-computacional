#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy

""" 
Rode cada linha em um terminal diferente
	roslaunch my_simulation caixas.launch
	rosrun aps4 aleatorio.py
"""

class Control():
	def __init__(self):
		self.rate = rospy.Rate(250) # 250 Hz

		# HSV Filter
		self.color_param = {
			"magenta": {
				"lower": ???,
				"upper": ???
			},
			"yellow": {
				"lower": ???,
				"upper": ???
			},
		}

		self.time = random.uniform(0.2, 2.0)
		self.target_time = 0

		# Subscribers
		self.bridge = CvBridge()
		self.odom_sub = rospy.Subscriber()
		self.laser_subscriber = rospy.Subscriber()
		self.image_sub = rospy.Subscriber()
		
		# Publishers
		self.cmd_vel_pub = rospy.Publisher()

		self.cmd_vel_pub.publish(Twist())

		self.selected_color = None
		self.robot_state = "rotate"
		self.robot_machine = {
			"rotate": self.rotate,
			"checar": self.checar,
			"center_on_coord": self.center_on_coord,
			"go_to_coord": self.go_to_coord,
			"para": self.para,
		}

		self.magenta_machine = {
			"aproxima": self.aproxima,
		}

		self.yellow_machine = {
			"afasta": self.afasta,
		}

	def odom_callback(self, data: Odometry):
		self.position = data.pose.pose.position
		
		orientation_list = [data.pose.pose.orientation.x,
							data.pose.pose.orientation.y,
							data.pose.pose.orientation.z,
							data.pose.pose.orientation.w]

		self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

		self.yaw = self.yaw % (2*np.pi)
	
	def laser_callback(self, msg: LaserScan) -> None:
		self.laser_msg = np.array(msg.ranges).round(decimals=2)
		self.laser_msg[self.laser_msg == 0] = np.inf

		self.laser_forward = ???
		self.laser_backwards = ???
	
	def color_segmentation(self, hsv: np.ndarray, lower_hsv: np.ndarray, upper_hsv: np.ndarray,) -> Point:
		""" 
		Use HSV color space to segment the image and find the center of the object.

		Args:
			bgr (np.ndarray): image in BGR format
		
		Returns:
			Point: x, y and area of the object
		"""
		...
		
	def image_callback(self, msg: CompressedImage) -> None:
		"""
		Callback function for the image topic
		"""
		try:
			cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
			hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		except CvBridgeError as e:
			print(e)
		
		...

	def rotate(self) -> None:
		"""
		Rotate the robot
		"""
		self.twist.angular.z = 0.5
		...

	def checar(self) -> None:
		"""
		Stop the robot
		"""
		if self.selected_color == "magenta":
			# append magenta_machine to robot_machine
			self.robot_machine.update(self.magenta_machine)
			self.robot_state = "aproxima"

		elif self.selected_color == "yellow":
			# append yellow_machine to robot_machine
			self.robot_machine.update(self.yellow_machine)
			self.robot_state = "afasta"

	def aproxima(self) -> None:
		"""
		Go to
		"""
		...

	def afasta(self) -> None:
		"""
		Go away
		"""
		...
	
	def para(self) -> None:
		"""
		Stop the robot
		"""
		...
	
	def center_on_coord(self):
		...

	def go_to_coord(self):
		...

	def control(self):
		'''
		This function is called at least at {self.rate} Hz.
		This function controls the robot.
		'''
		self.twist = Twist()
		print(f'self.robot_state: {self.robot_state}')
		self.robot_machine[self.robot_state]()

		self.cmd_vel_pub.publish(self.twist)
		
		self.rate.sleep()

def main():
	rospy.init_node('Aleatorio')
	control = Control()
	rospy.sleep(1)

	while not rospy.is_shutdown():
		control.control()

if __name__=="__main__":
	main()

