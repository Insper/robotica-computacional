from std_msgs.msg import Float64
import time

class Garra(): # Mixin: não herda de Node. Use junto com um Node: class MeuNo(Node, Garra)

    def __init__(self, delay: float = 2.0):
        # Chame DEPOIS de Node.__init__(self, ...): create_publisher vem do nó.
        self.delay = delay # Tempo de espera (s) após cada movimento

        # Publishers (criados no próprio nó que herda de Garra)
        self.ombro_pub = self.create_publisher(
            Float64,
            "/joint1_position_controller/command",
            10
        )
        self.garra_pub = self.create_publisher(
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
