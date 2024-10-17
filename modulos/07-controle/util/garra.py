from std_msgs.msg import Float64
import numpy as np
import time

class Garra(): # Mude o nome da classe

    def __init__(self):
        # Inicialização de variáveis

        self.delay = 1.0
        
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
        if command == 'open':
            self.garra.publish(-1.0)
            self.garra.publish(-1.0)
        elif command == 'close':
            self.garra.publish(0.0)
            self.garra.publish(0.0)
        elif command == 'up':
            self.ombro.publish(1.5)
            self.ombro.publish(1.5)
        elif command == 'mid':
            self.ombro.publish(0.0)
            self.ombro.publish(0.0)
        elif command == 'down':
            self.ombro.publish(-1.0)
            self.ombro.publish(-1.0)
        time.sleep(self.delay)

