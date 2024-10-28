from std_msgs.msg import Float64
import numpy as np
import time

class Garra(): # Mude o nome da classe

    def __init__(self):
        # Inicialização de variáveis

        self.delay = 2.0
        
        # Publishers
        self.ombro_pub = self.create_publisher(
            Float64,
            "/joint1_position_controller/command",
            10
        )
        self.ombro_pub = self.create_publisher(
            Float64,
            "/joint2_position_controller/command",
            10
        )

    def controla_garra(self, command: str):
        msg = Float64()
        if command == 'open':
            msg.data = -1.0
            self.garra.publish(msg)
            self.garra.publish(msg)
        elif command == 'close':
            msg.data = 0.0
            self.garra.publish(msg)
            self.garra.publish(msg)
        elif command == 'up':
            msg.data = 1.5
            self.ombro.publish(msg)
            self.ombro.publish(msg)
        elif command == 'mid':
            msg.data = 0.0
            self.ombro.publish(msg)
            self.ombro.publish(msg)
        elif command == 'down':
            msg.data = -1.0
            self.ombro.publish(msg)
            self.ombro.publish(msg)
        time.sleep(self.delay)

