#!/usr/bin/env python3
# -*- coding:utf-8 -*-


#   exemplo adaptado do livro:
#   
#  Programming Robots with ROS.
#  A Practical Introduction to the Robot Operating System
#  Example 12-5. follower_p.py pag265
#  
#  Referendia PD:https://github.com/martinohanlon/RobotPID/blob/master/mock/mock_robot_pd.py 

import rospy
import numpy as np
import math
import cv2
import time
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class FollowerLinha:

    def __init__(self):
        
        self.bridge = CvBridge()
        self.cv_image = None
	    #topico da camera do robo real
	    #self.image_sub = rospy.Subscriber('/v4l/camera/image_raw/compressed',
        #topico da camera do robo simulado
        self.image_sub = rospy.Subscriber('/camera/image/compressed',
                                            CompressedImage, 
                                            self.image_callback, 
                                            queue_size=4, 
                                            buff_size = 2**24)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_linha',
                                             Twist, 
                                             queue_size=1)

        self.ve_linha_pub = rospy.Publisher('/ve_linha',
                                            Bool,
                                            queue_size=1)
        
        self.twist = Twist()
        
        # Variaveis de controle
        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1
        self.angulo = 0

        # Variaveis de estado
        self.ve_linha = False
        
        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)
    
    def image_callback(self, msg):
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
	    #cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real
	
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([20, 50, 50],dtype=np.uint8)
            upper_yellow = np.array([50, 255, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            h, w, d = cv_image.shape
            search_top = 3*h//4
            search_bot = 3*h//4 + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0

            self.w = w
            self.h = h

            M = cv2.moments(mask)

            if M['m00'] > 0:
                self.ve_linha = True
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
                cv2.circle(cv_image, (self.cx, self.cy), 20, (0,0,255), -1)
            else:
                self.ve_linha = False

            cv2.imshow("window", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)
    
    def segue_linha(self):
        ### BEGIN CONTROL
        err = self.cx - self.w/2
        #------controle P simples--------------------
    
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100

        self.rate.sleep()

    def control(self):
        # Publicar velocidae e a variavel de estado
        self.segue_linha()
        self.cmd_vel_pub.publish(self.twist)
        self.ve_linha_pub.publish(Bool(self.ve_linha))

# Main loop
if __name__=="__main__":
    rospy.init_node('follower_linha')
    follower = FollowerLinha()

    while not rospy.is_shutdown():
        follower.control()

# END ALL
