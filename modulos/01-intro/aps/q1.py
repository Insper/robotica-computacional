import time
from util import Mapa

class Control(): # Herdando de Mapa
    def __init__(self):
        # Inicializa a classe Pai

        self.robot_state = 'stop'
        self.state_machine = {
            'forward': self.forward,
            'left': self.left,
            'right': self.right,
            'stop': self.stop,
        }
    
    def forward(self) -> None:
        # Move pra cima subtraindo 1 de i
        # Chama o método de atualização de posição
        pass

    def left(self) -> None:
        # Move pra esquerda subtraindo 1 de j
        # Chama o método de atualização de posição
        pass

    def right(self) -> None:
        # Move pra direita somando 1 a j
        # Chama o método de atualização de posição
        pass
    
    def stop(self) -> None:
        # Não faz nada
        pass

    def control(self) -> None:
        # A lógica de controle do carro deve mudar o estado do carro (self.robot_state)
        # Não chame direto os métodos de movimento, mas sim mude o estado do carro.

        # Verifique se a posição acima está livre, se sim, mude o estado para 'forward'.
        # Se não, verifique se a posição à esquerda ou à direita está livre, se sim, mude o estado para 'left' ou 'right'.
        # Pare quando estiver na primeira linha mudando o estado para 'stop'.

        # IMPORTANTE: Certifique-se de que o carro não saia dos limites do mapa.
        
        # Chamada do método de movimento a partir do dicionário
        self.state_machine[self.robot_state]()

        # Mostra a grade atual
        self.mostrar()
def main():
    control = Control()
    control.mostrar()

    i = 40
    
    while not control.robot_state == 'stop' and i > 0:
        control.control()
        time.sleep(1)
        i -= 1

if __name__=="__main__":
    main()