from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
import numpy as np
import rclpy

class Laser(): # Mude o nome da classe
    def __init__(self):

        print("Laser Inciado")

        # Inicialização de variáveis
        self.front = [0]
        self.openning = 5
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        rclpy.spin_once(self, timeout_sec=1.0) # Executa uma vez para pegar a primeira leitura

    def custom_laser(self):
        pass
    
    def laser_callback(self, data: LaserScan):
        self.laser_msg = np.array(data.ranges).round(decimals=2)
        self.laser_msg[self.laser_msg == 0] = np.inf
        self.laser_msg = list(self.laser_msg)

        # +- 5 degrees
        self.front = self.laser_msg[:self.openning] + self.laser_msg[-self.openning:]
        self.left = self.laser_msg[90-self.openning:90+self.openning]
        self.right = self.laser_msg[275-self.openning:275+self.openning]
        self.back = self.laser_msg[180-self.openning:180+self.openning]

        self.custom_laser()


