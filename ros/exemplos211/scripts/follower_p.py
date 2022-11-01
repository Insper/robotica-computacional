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

class Follower:

    def __init__(self):
        
        self.bridge = CvBridge()
        self.cv_image = None
        self.image_sub = rospy.Subscriber('/v4l/camera/image_raw/compressed',
                                            CompressedImage, 
                                            self.image_callback, 
                                            queue_size=4, 
                                            buff_size = 2**24)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                            Twist, 
                                            queue_size=1)
        
        self.laser_subscriber = rospy.Subscriber('/scan',
                                                 LaserScan, 
			                                    self.laser_callback)
        
        self.twist = Twist()
        self.laser_msg = LaserScan()
        
        self.lastError = 0
        self.max_vel_linear = 0.2
        self.max_vel_angular = 2.0
        self.herts = 250
        self.rate = rospy.Rate(self.herts)

    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_laser(self, pos):
        return self.laser_msg.ranges[pos]
    
    def image_callback(self, msg):
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
            cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real

            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([12, 50, 50],dtype=np.uint8)
            upper_yellow = np.array([40, 255, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            h, w, d = cv_image.shape
            search_top = 3*h/4
            search_bot = 3*h/4 + 20
            mask[0:int(search_top), 0:w] = 0
            mask[int(search_bot):h, 0:w] = 0
            
            M = cv2.moments(mask)

            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)

                ### BEGIN CONTROL
                err = cx - w/2
                #------controle P simples--------------------
            
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(err) / 100
           
                #------Controle PD simples----------------- tem que validar 
                                
            #    kp = 0.2 # ganho P para tunar
            #    kd = 0.03 # ganoh D para tunar
            #    
            #    proporcional = kp *err
            #    derivativo = kd*(err - self.lastError)/(1.0/self.herts)
            #    PD = proporcional + derivativo
            #    
            #    self.lastError = err
            #
            #    self.twist.linear.x =min(self.max_vel_linear*((1-abs(err)/(w/2))**2), self.max_vel_linear)
            #    self.twist.angular.z = -max(PD/10, -self.max_vel_angular) if PD/10 < 0 else -min(PD/10, self.max_vel_angular) # anti-windup
                
                ### END CONTROL

                #publica velocidade
                self.cmd_vel_pub.publish(self.twist)
                rospy.loginfo("linear: %f angular: %f", self.twist.linear.x, self.twist.angular.z)
                self.rate.sleep()

            cv2.imshow("window", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL