from rclpy.node import Node
from std_msgs.msg import Float64
import time

class Garra(): # Não herda de Node: usa o nó que a criou

    def __init__(self, node: Node, delay: float = 2.0):
        # Inicialização de variáveis
        self.node = node
        self.delay = delay # Tempo de espera (s) após cada movimento

        # Publishers (criados no nó recebido)
        self.ombro_pub = node.create_publisher(
            Float64,
            "/joint1_position_controller/command",
            10
        )
        self.garra_pub = node.create_publisher(
            Float64,
            "/joint2_position_controller/command",
            10
        )

    def controla_garra(self, command: str):
        # Escolhe o publisher e o valor antes de publicar ou esperar
        if command == 'abrir':
            publisher, valor = self.garra_pub, -1.0
        elif command == 'fechar':
            publisher, valor = self.garra_pub, 0.0
        elif command == 'cima':
            publisher, valor = self.ombro_pub, 1.5
        elif command == 'frente':
            publisher, valor = self.ombro_pub, 0.0
        elif command == 'baixo':
            publisher, valor = self.ombro_pub, -1.0
        else:
            raise ValueError(f"Comando desconhecido para a garra: {command!r}")

        msg = Float64()
        msg.data = valor
        publisher.publish(msg)
        time.sleep(self.delay) # Bloqueia: o nó não processa callbacks nesse tempo
