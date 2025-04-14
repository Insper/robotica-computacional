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
        # Move subtraindo 1 uma linha
        # Atualiza a posição
        pass

    def left(self) -> None:
        # Move subtraindo 1 uma coluna
        # Atualiza a posição
        pass

    def right(self) -> None:
        # Move somando 1 uma coluna
        # Atualiza a posição
        pass
    
    def stop(self) -> None:
        # Não faz nada
        pass

    def control(self) -> None:
        # Verifique se a posição acima está livre, se sim, mova para cima.
        # Se não, verifique se a posição à esquerda ou à direita está livre, se sim, mova para um dos lados.
        # Pare quando estiver na primeira linha.
        
        # Chamada do método de movimento a partir do dicionário
        # self.state_machine...

        # Mostre a grade atual
        
        pass
        
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